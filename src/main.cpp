#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 4  //for mpu6050

Adafruit_BMP280 bmp; // I2C
MPU6050 mpu;
TaskHandle_t th[2]; // Event Handle
HardwareSerial COMM(1);
HardwareSerial GPS(2);

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float g_Temperature = 0.0;
float g_Pressure = 0.0;
float g_Altitude = 0.0;
float g_yaw = 0.0, g_pitch = 0.0, g_roll = 0.0;

String g_receivedcommand = "";
char g_command ='/';
char g_oldcommand = '/'; // for debug
char g_oldtestcommand = '/'; // for debug

int g_loop0counter = 1;

//Global variables of the launch test in test mode
int g_launchcounter = 0;

//Global variables of the wingalt test in test mode
int g_wingaltcounter = 0;
float g_wingaltheight[5];
float g_wingaltnewaverage = -100.0;
float g_wingaltoldaverage = -100.0;

int16_t g_ax, g_ay, g_az;
int16_t g_gx, g_gy, g_gz;

QueueHandle_t queue_ax;
QueueHandle_t queue_ay;
QueueHandle_t queue_az;
QueueHandle_t queue_magnitude;
QueueHandle_t queue_height;

int counter = 0;

enum systemPhase{
    PHASE_WAIT,
    PHASE_TEST,
    PHASE_CONFIG,
    PHASE_LAUNCH,
    PHASE_RISE,
    PHASE_GLIDE,
    PHASE_SPLASHDOWN,
    PHASE_EMERGENCY,
    PHASE_STAND
} g_Phase;

enum systemTest{
    TEST_FLIGHTMODE,
    TEST_LAUNCH,
    TEST_WINGALT,
    TEST_WINGTIMER,
    TEST_STAND
} g_Test;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady();
systemPhase phaseDecide(char g_command, systemPhase oldPhase);
systemTest testDecide(char testcommand, systemTest oldTest);
//void sendEmergency();
//float checkMpu();


void loop0(void* pvParameters);
void loop1(void* pvParameters);

void setup() {
    //initialize both Serial ports
    Serial.begin(115200);
    COMM.begin(115200, SERIAL_8N1, 18,19);
    GPS.begin(115200);

    //initialize MPU6050
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000L); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu.setRate(9); //set sampling rate 100Hz
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); //set accel range +-16g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000); // ser gyro range 1000 deg/sec(dps)
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    //Serial.printlnSerial.println(F("BMP280 test"));
    if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    //initialize the g_Phase state
    g_Phase = PHASE_WAIT;

    // create a queue for inter-core data sharing
    queue_ax = xQueueCreate(10, sizeof(float));
    if(queue_ax == NULL){
        Serial.println("can NOT create QUEUE_AX");
    }
    queue_ay = xQueueCreate(10, sizeof(float));
    if(queue_ay == NULL){
        Serial.println("can NOT create QUEUE_AY");
    }
    queue_az = xQueueCreate(10, sizeof(float));
    if(queue_az == NULL){
        Serial.println("can NOT create QUEUE_AZ");
    }
    queue_magnitude = xQueueCreate(10, sizeof(float));
    if(queue_magnitude == NULL){
        Serial.println("can NOT create QUEUE_MAGNITUDE");
    }
    queue_height = xQueueCreate(20, sizeof(float));
    if(queue_height == NULL){
        Serial.println("can NOT create QUEUE_HEIGHT");
    }

    // initial setting of light sleep
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    gpio_pullup_en(GPIO_NUM_33);
    gpio_pulldown_dis(GPIO_NUM_33);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);

    xTaskCreatePinnedToCore(loop0, "Loop0", 8192, NULL, 2, &th[0], 0); // loop0 manipulate g_Phase dicision
    delay(500);
    xTaskCreatePinnedToCore(loop1, "Loop1", 8192, NULL, 2, &th[1], 1); // loop1 manipulate sensor processing
    delay(500);
}

/*  loop0 manipulate g_Phase dicision  */
void loop0 (void* pvParameters){
    while(1){
        //Serial.println("loop0 is working");
        switch (g_Phase)
        {
            case PHASE_WAIT:
            {
                Serial.println("After 5 seconds enter light sleep mode");
                delay(5000);
                esp_light_sleep_start();
                Serial.println("I woke up");
                break;
            }

            case PHASE_TEST:
            {
                Serial.println("Enter TEST Sequence");

                char testcommand = g_receivedcommand[1];
                Serial.print(testcommand);
                Serial.println(" test g_command received.");

                switch(g_Test)
                {
                    case TEST_FLIGHTMODE:
                    {   
                        g_Phase = PHASE_LAUNCH;
                        Serial.println("[TEST] Entry LAUNCH sequence [TEST]");
                        break;
                    }

                    case TEST_LAUNCH:
                    {
                        int counter_launch = 0;
                        Serial.println("[TEST] Start the LAUNCH test [TEST]");

                        float magnitude;
                        xQueueReceive(queue_magnitude, &magnitude, portMAX_DELAY);

                        //Truncate the first 5 thousand times
                        while(g_loop0counter > 0 && g_loop0counter < 5000){
                            Serial.println("Waiting for the value to stabilize");
                            g_loop0counter++;
                            vTaskDelay(1);
                        }

                        if(g_loop0counter >= 5000 && magnitude > 25000){
                            if((g_loop0counter - g_launchcounter) == 1){ //Continuous judgment
                                counter_launch++;
                            }
                            g_launchcounter = g_loop0counter;
                        }

                        if(counter_launch >= 5){
                            Serial.println("[TEST] Ready for launch [TEST]");
                        }else{
                            Serial.println("[TEST] NOT Ready for launch [TEST]");
                        }
                        break;

                        
                    }

                    case TEST_WINGALT:
                    {
                        int counter_wingalt = 0;
                        int validation = 5;

                        Serial.println(g_loop0counter);
                        if(g_loop0counter == 1) Serial.println("[TEST] Start Verify the wing expansion (ALT) [TEST]");

                        //Obtain moving g_wingaltnewaverage
                        while(g_wingaltcounter < 5){
                            g_wingaltheight[g_wingaltcounter] = bmp.readAltitude(1013.25);
                            g_wingaltcounter++;
                        }

                        for(int i = (validation - 1); i > 0; i--) g_wingaltheight[i] = g_wingaltheight[i-1];
                        g_wingaltheight[0] = bmp.readAltitude(1013.25);

                        g_wingaltnewaverage = 0.0;
                        for(int i = 0; i < validation; i++) g_wingaltnewaverage += g_wingaltheight[i];
                        g_wingaltnewaverage = g_wingaltnewaverage / float(validation);        
                        Serial.println(g_wingaltnewaverage);          

                        if(g_wingaltnewaverage < g_wingaltoldaverage){
                            if((g_loop0counter - g_wingaltcounter) == 1){ //Continuous judgment
                                counter_wingalt++;
                            }
                            g_wingaltcounter = g_loop0counter;
                        }
                        g_wingaltoldaverage = g_wingaltnewaverage;

                        if(counter_wingalt >= 5){
                            Serial.println("Ready for expand the wing");
                        }else{
                            Serial.println("NOT Ready for expand the wing");
                        }
                        break;
                    }

                    case TEST_WINGTIMER:
                    {
                        break;
                    }


                    case TEST_STAND:
                    {
                        Serial.println("TEST_STAND");
                        break;
                    }


                    default:
                    {
                        break;
                    }

                }

            }

            case PHASE_CONFIG:
            {
                break;
            }

            case PHASE_LAUNCH:
            {
                break;
            }

            case PHASE_RISE:
            {
                break;
            }

            case PHASE_GLIDE:
            {
                break;
            }

            case PHASE_SPLASHDOWN:
            {
                break;
            }

            case PHASE_EMERGENCY:
            {
                Serial.println("EMERGENCY Phase");
                //sendEmergency();
                break;
            }

            case PHASE_STAND:
            {
                Serial.println("PHASE_STAND");
                break;
            }
        
            default:
            {
                break;
            }
        }

        // wait for g_command
        if(COMM.available() > 0){
            g_receivedcommand = COMM.readStringUntil('\n');
            Serial.print(g_receivedcommand);
            Serial.println(" g_receivedcommand received.");
            g_command = g_receivedcommand[0];
            Serial.print(g_command);
            Serial.println(" received.");

            //In test mode the first letter is the same
            if(g_command == 't' || g_command != g_oldcommand){
                g_Phase = phaseDecide(g_command, g_Phase);
            }else{
                g_Phase = PHASE_STAND;
            }
            g_oldcommand = g_command;
        }

        //debug status
        Serial.print("g_oldcommand: ");
        Serial.print(g_oldcommand);
        Serial.print(", g_oldtestcommand: ");
        Serial.println(g_oldtestcommand);

        vTaskDelay(1);
    }
    g_loop0counter++;
    
}

/*  loop1 manipulate sensor processing  */
void loop1 (void* pvParameters){
    while(1){
        //Serial.println("loop1 is working");

        // get sensor value
        // from BMP280
        g_Temperature = bmp.readTemperature();
        g_Pressure = bmp.readPressure();
        g_Altitude = bmp.readAltitude(1013.25);
        // from MPU6050

        // if programming failed, don't try to do anything
        if (!dmpReady){
            Serial.println("!!! Emergency situation occurred !!!");
            g_Phase = PHASE_EMERGENCY;
        }

        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {
            if (mpuInterrupt && fifoCount < packetSize) {
            // try to get out of the infinite loop 
            fifoCount = mpu.getFIFOCount();
            }  
        }

        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            fifoCount = mpu.getFIFOCount();
            Serial.println(F("FIFO overflow!"));
        } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;

            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                
                /*
                Serial.print("ypr\t");
                Serial.print(ypr[0] * 180/M_PI);
                Serial.print("\t");
                Serial.print(ypr[1] * 180/M_PI);
                Serial.print("\t");
                Serial.println(ypr[2] * 180/M_PI);
                */

                mpu.getMotion6(&g_ax, &g_ay, &g_az, &g_gx, &g_gy, &g_gz);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                float magnitude = aa.getMagnitude();
                xQueueSend(queue_ax, &g_ax, portMAX_DELAY);
                xQueueSend(queue_ay, &g_ay, portMAX_DELAY);
                xQueueSend(queue_az, &g_az, portMAX_DELAY);
                xQueueSend(queue_magnitude, &magnitude, portMAX_DELAY);
                xQueueSend(queue_height, &g_Altitude, portMAX_DELAY);
                
                g_yaw = ypr[0] * 180/M_PI;
                g_pitch = ypr[1] * 180/M_PI;
                g_roll = ypr[2] * 180/M_PI;
            #endif
        }

        /*
        Serial.print(g_Temperature);
        Serial.print("\t");
        Serial.print(g_Pressure);
        Serial.print("\t");
        Serial.println(g_Altitude);
        */

        vTaskDelay(1);
    }
    
}

void loop() {
}

void dmpDataReady() {
    mpuInterrupt = true;
}

systemPhase phaseDecide(char g_command, systemPhase oldPhase){
    systemPhase newPhase = PHASE_STAND;
    if(g_command == 'w') return PHASE_WAIT;
    if(g_command == 't') return PHASE_TEST;
    if(g_command == 'c') return PHASE_CONFIG;
    if(g_command == 'l') return PHASE_LAUNCH;
    if(g_command == 'r') return PHASE_RISE;
    if(g_command == 'g') return PHASE_GLIDE;
    if(g_command == 's') return PHASE_SPLASHDOWN;
    if(g_command == 'e') return PHASE_EMERGENCY;
    return oldPhase;
}

systemTest testDecide(char testcommand, systemTest oldTest){
    systemTest newTest = TEST_STAND;
    if(testcommand == '0') return TEST_FLIGHTMODE;
    if(testcommand == '1') return TEST_LAUNCH;
    if(testcommand == '2') return TEST_WINGALT;
    if(testcommand == '3') return TEST_WINGTIMER;
    return oldTest;
}

/*
void sendEmergency(){
    // TODO: implement send EMG data to MKRWAN1300 
}
*/