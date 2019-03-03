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

String g_receivedcommand = "";
char g_command ='/';
char g_testcommand = '/';
char g_oldcommand = '/'; // for debug

//Global variables of the loop0
int g_loop0counter = 1;

//Global variables of the launch test in test mode
int g_launchcounter = 0;

//Global variables of the wingalt test in test mode
int g_wingcounter = 0;
float g_wingheight[5];
float g_wingnewaverage = -100.0;
float g_wingoldaverage = -100.0;

//for the wingXXX test
bool successflag_timer = false;
bool phaselock = false;

int16_t g_ax, g_ay, g_az;
int16_t g_gx, g_gy, g_gz;

QueueHandle_t queue_magnitude;
QueueHandle_t queue_altitude;

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
bool launchDecide(int magnitudecriterion, int countercriterion);
bool wingaltDecide(int countercriterion);

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
    queue_magnitude = xQueueCreate(512, sizeof(float));
    if(queue_magnitude == NULL){
        Serial.println("can NOT create QUEUE_MAGNITUDE");
    }
    queue_altitude = xQueueCreate(512, sizeof(float));
    if(queue_altitude == NULL){
        Serial.println("can NOT create QUEUE_HEIGHT");
    }

    // initial setting of light sleep
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    gpio_pullup_en(GPIO_NUM_33);
    gpio_pulldown_dis(GPIO_NUM_33);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);

    xTaskCreatePinnedToCore(loop0, "Loop0", 8192, NULL, 2, &th[0], 0); // loop0 manipulate g_Phase dicision
    vTaskDelay(500);
    xTaskCreatePinnedToCore(loop1, "Loop1", 8192, NULL, 2, &th[1], 1); // loop1 manipulate sensor processing
}

/*  loop0 manipulate g_Phase dicision  */
void loop0 (void* pvParameters){
    int64_t starttime;
    while(1){
        g_loop0counter++;
        TickType_t starttick = xTaskGetTickCount();
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
                //Serial.println("Enter TEST Sequence");

                char testcommand = g_receivedcommand[1];
                g_Test = testDecide(testcommand, g_Test);
                
                /* debug status */
                Serial.print(testcommand);
                Serial.print(" testcommand received. So system decided ");
                Serial.print(g_Test);
                Serial.println(" TEST.");

                switch(g_Test)
                {
                    case TEST_FLIGHTMODE:
                    {   
                        Serial.println("[TEST] Entry LAUNCH sequence [TEST]");
                        break;
                    }

                    case TEST_LAUNCH:
                    {
                        if(launchDecide(25000,5)){
                            Serial.println("[TEST] READY for launch [TEST]");
                        }else{
                            Serial.println("[TEST] NOT READY for launch [TEST]");
                        }
                        break;
                    }

                    case TEST_WINGALT:
                    {
                        if(wingaltDecide(5)){
                            Serial.println("[TEST] READY for expand the wing [TEST]");
                        }else{
                            Serial.println("[TEST] NOT READY for expand the wing [TEST]");
                        }
                        break;
                    }

                    case TEST_WINGTIMER:
                    {
                        if(g_loop0counter < 10){
                            Serial.println("[TEST] Start Verify the wing expansion (TIMER) [TEST]");
                            starttime = esp_timer_get_time();
                        }
                        Serial.println("ENTRY : TEST_WINGTIMER");
                        //Mostly wingalt's copy (except for timer condition)
                        //Activate the timer at the same time as the launch judgment
                        int counter_wingtimer = 0;;
                        float altitude;

                        int64_t entrytime = esp_timer_get_time();
                        int64_t elapsedtime = entrytime - starttime;
                        Serial.print("elapsed time is ");
                        Serial.printf("%"PRId64"\n",elapsedtime);
                        if(elapsedtime < 10000000){
                            if(wingaltDecide(5)){
                                Serial.println("[TEST] Ready for expand the wing [TEST]");
                                successflag_timer = true;
                            }else{
                                Serial.println("[TEST] NOT Ready for expand the wing [TEST]");
                            }

                            break;

                        }else if(!successflag_timer && elapsedtime >= 10000000){
                            Serial.println("[TEST] FORCE : expand the wing [TEST]");
                            break;
                        }
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

                break;
                }

            }

            case PHASE_CONFIG:
            {
                break;
            }

            case PHASE_LAUNCH: //launch determination
            {
                if(launchDecide(30000,10)){
                    Serial.println("[FLIGHT] READY for launch [FLIGHT]");
                }else{
                    Serial.println("[FLIGHT] NOT READY for launch [FLIGHT]");
                }
                break;   
            }

            case PHASE_RISE:
            {
                int64_t entrytime = esp_timer_get_time();
                int64_t elapsedtime = entrytime - starttime;
                if(elapsedtime >= 7000 && elapsedtime < 16000){
                    if(wingaltDecide(5)){
                        Serial.println("[FLIGHT] Ready for expand the wing [FLIGHT]");
                        g_Phase = PHASE_GLIDE;
                        successflag_timer = true;
                    }else{
                        Serial.println("[FLIGHT] NOT Ready for expand the wing [FLIGHT]");
                    }
                }else if(elapsedtime >= 16000){
                        Serial.println("[FLIGHT] FORCE : expand the wing [FLIGHT]");
                        g_Phase = PHASE_GLIDE;
                        successflag_timer = true;
                }
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
        if(!phaselock && COMM.available() > 0){
            g_receivedcommand = COMM.readStringUntil('\n');
            //Serial.print(g_receivedcommand);
            //Serial.println(" g_receivedcommand received.");
            g_command = g_receivedcommand[0];
            //Serial.print(g_command);
            //Serial.print(" received.");

            /*
            //In test mode the first letter is the same
            if(g_command == 't' || g_command != g_oldcommand){
                g_Phase = phaseDecide(g_command, g_Phase);
            }else{
                //g_Phase = PHASE_STAND;
            }
            */
            g_Phase = phaseDecide(g_command, g_Phase);
            g_oldcommand = g_command;
        }

        //debug status
        //Serial.print("g_oldcommand: ");
        //Serial.println(g_oldcommand);

        TickType_t endtick = xTaskGetTickCount();
        TickType_t executiontick = endtick - starttick;
        Serial.print("Execution tick (LOOP0) is ");
        Serial.println(executiontick);

        vTaskDelay(30); //Need to be adjusted
    }
}

/*  loop1 manipulate sensor processing  */
void loop1 (void* pvParameters){
    while(1){
        TickType_t starttick = xTaskGetTickCount(); 
        //int64_t starttime = esp_timer_get_time();
        Serial.print("fifoCount at first are ");
        Serial.printf("%"PRId16"\n",fifoCount);
        Serial.println("loop1 is working");

        // get sensor value
        // from BMP280
        float altitude = bmp.readAltitude(1013.25);

        // if programming failed, don't try to do anything
        if (!dmpReady){
            Serial.println("!!! EMERGENCY EMERGENCY EMERGENCY !!!");
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

            mpu.dmpGetAccel(&aa, fifoBuffer);
            float magnitude = aa.getMagnitude();

            BaseType_t xStatus_magnitude = xQueueSend(queue_magnitude, &magnitude, 0);
            if(xStatus_magnitude == pdTRUE){
                Serial.println("SUCCESS: send MAGNITUDE");
            }else{
                Serial.println("FAILED: send MAGNITUDE");
            }

            BaseType_t xStatus_height = xQueueSend(queue_altitude, &altitude, 0);
            if(xStatus_height == pdTRUE){
                Serial.println("SUCCESS: send HEIGHT");
            }else{
                Serial.println("FAILED: send HEIGHT");
            }

        }

        TickType_t endtick = xTaskGetTickCount();
        TickType_t executiontick = endtick - starttick;
        Serial.print("Execution tick (LOOP1) is ");
        Serial.println(executiontick);

        Serial.print("fifoCount at end are ");
        Serial.printf("%"PRId16"\n",fifoCount);
        vTaskDelay(30); //Need to be adjusted
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

bool launchDecide(int magnitudecriterion, int countercriterion){
    int counter_launch = 0;
    //Serial.println("ENTRY : LAUNCHDECIDE");
    Serial.println(g_loop0counter);

    float magnitude = 0.0;
    BaseType_t xStatus = xQueueReceive(queue_magnitude, &magnitude, 0);
    if(xStatus == pdTRUE){
        Serial.println("SUCCESS : received TEST_LAUNCH");
    }else{
        Serial.println("FAILED : received TEST_LAUNCH");
    }
    //Serial.println(xStatus);
    Serial.print("magnitude is ");
    Serial.println(magnitude);

    if(g_loop0counter >= 5000 && magnitude > magnitudecriterion){
        if((g_loop0counter - g_launchcounter) == 1){ //Continuous judgment
            counter_launch++;
        }
        g_launchcounter = g_loop0counter;
    }

    if(counter_launch >= countercriterion){
        return true;
    }else{
        return false;
    }
}

bool wingaltDecide(int countercriterion){
    //Serial.println("ENTRY : WINGALTDECIDE");
    int counter_wingalt = 0;
    int validation = 5;
    float altitude;

    Serial.println(g_loop0counter);
    for(int i = (validation - 1); i > 0; i--) g_wingheight[i] = g_wingheight[i-1];

    BaseType_t xStatus = xQueueReceive(queue_altitude, &altitude, 0);
    //Serial.println(altitude);
    if(xStatus == pdTRUE){
        Serial.println("SUCCESS : receive TEST_WINGALT");
    }else{
        Serial.println("FAILED : receive TEST_WINGALT");
        
    }

    if(g_wingcounter < 5){
        g_wingheight[g_wingcounter] = altitude;
        g_wingcounter++;
    }else{
        g_wingheight[0] = altitude;
    }

    g_wingnewaverage = 0.0;
    for(int i = 0; i < validation; i++) g_wingnewaverage += g_wingheight[i];
    g_wingnewaverage = g_wingnewaverage / float(validation);
    Serial.print("Average is ");      
    Serial.println(g_wingnewaverage);          

    if(g_wingnewaverage < g_wingoldaverage){
        if((g_loop0counter - g_wingcounter) == 1){ //Continuous judgment
            counter_wingalt++;
        }
        g_wingcounter = g_loop0counter;
    }
    g_wingoldaverage = g_wingnewaverage;

    if(counter_wingalt >= countercriterion){
        return true;
    }else{
        return false;
    }
}