#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>

#define INTERRUPT_PIN 4  //mpu6050の割り込みピン

Adafruit_BMP280 bmp;
MPU6050 mpu;
TaskHandle_t th[2];
HardwareSerial COMM(1);
HardwareSerial GPS(2);

bool dmpReady = false;  // DMPの初期化が成功したときにtrueになる
uint8_t mpuIntStatus;   // MPUからの実際の割り込みステータスバイトを保持する
uint8_t devStatus;      // それぞれのデバイスの処理後のステータスを返す(0 = 成功, !0 = エラー)
uint16_t packetSize;    // DMPパケットサイズを格納する(デフォルトでは42バイト)
uint16_t fifoCount;     // FIFOにある現在のすべてのバイトを格納
uint8_t fifoBuffer[64]; // FIFO格納バッファ

VectorInt16 aa;         // 加速度格納用

String g_receivedcommand = "";
char g_command ='/';
char g_testcommand = '/';
char g_oldcommand = '/'; // for debug

//ループカウンタ(loop0)
int g_loop0counter = 1;

//ループカウンタ(TEST_LAUNCH)
int g_launchcounter = 0;

//ループカウンタ(TEST_WINGALT,TEST_WINGTIMER)
int g_wingcounter = 0;

//移動平均算出用
float g_wingheight[5];
float g_wingnewaverage = -100.0;
float g_wingoldaverage = -100.0;

bool successflag_timer = false;

//コマンド間違い防止のためにフェーズをロックする
bool phaselock = false;

QueueHandle_t queue_magnitude;
QueueHandle_t queue_altitude;

int counter = 0;

// サーボチャンネル、ピン設定
const int CHANNEL0 = 0;
const int CHANNEL1 = 1;
const int CHANNEL2 = 2;
const int ROTATESERVO1 = 25;
const int ROTATESERVO2 = 26;
const int SERVO1 = 27;

// フェーズ定義
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

// テストモード定義
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
    //UART0~2の設定
    Serial.begin(115200);
    COMM.begin(115200, SERIAL_8N1, 18,19);
    GPS.begin(115200);

    //MPU6050の初期化
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000L); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // MPU6050の追加設定
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu.setRate(9); //set sampling rate 100Hz
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); //set accel range +-16g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000); // ser gyro range 1000 deg/sec(dps)
    pinMode(INTERRUPT_PIN, INPUT);

    // MPU6050接続確認
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // 開始まで待機
    Serial.println(F("\nSend any character to begin DMP: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // DMPの初期化
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // MPU6050の各軸にオフセットを与える
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // データシートでは1688

    if (devStatus == 0) {
        // DMP起動
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // esp32の割り込み検知を有効にする
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // DMPの使用可能の如何を通達
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        //  後の処理のためにFIFOのパケットを取得
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // code
        // 1 = 初期メモリロード失敗
        // 2 = DMP設定のアップデート失敗
        // (センサが壊れてたら大抵は1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
    }

    /* データシートでのデフォルトを設定 */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* 実行モード */
                    Adafruit_BMP280::SAMPLING_X2,     /* 温度をオーバーサンプリング */
                    Adafruit_BMP280::SAMPLING_X16,    /* 圧力をオーバーサンプリング */
                    Adafruit_BMP280::FILTER_X16,      /* フィルター */
                    Adafruit_BMP280::STANDBY_MS_500); /* 待機時間の設定 */

    // フェーズを初期化
    g_Phase = PHASE_WAIT;

    // コア間データ共有用のキューを設定
    queue_magnitude = xQueueCreate(512, sizeof(float));
    if(queue_magnitude == NULL){
        Serial.println("can NOT create QUEUE_MAGNITUDE");
    }
    queue_altitude = xQueueCreate(512, sizeof(float));
    if(queue_altitude == NULL){
        Serial.println("can NOT create QUEUE_HEIGHT");
    }

    // PWMサーボの設定
    ledcSetup(CHANNEL0, 50, 10);
    ledcSetup(CHANNEL1, 50, 10);
    ledcSetup(CHANNEL2, 50, 10);
    ledcAttachPin(ROTATESERVO1, 0);
    ledcAttachPin(ROTATESERVO2, 1);
    ledcAttachPin(SERVO1, 2);

    // light sleepの初期化
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    gpio_pullup_en(GPIO_NUM_33);
    gpio_pulldown_dis(GPIO_NUM_33);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);

    // デュアルコアの設定
    xTaskCreatePinnedToCore(loop0, "Loop0", 8192, NULL, 2, &th[0], 0); // loop0 manipulate g_Phase dicision
    vTaskDelay(500);
    xTaskCreatePinnedToCore(loop1, "Loop1", 8192, NULL, 2, &th[1], 1); // loop1 manipulate sensor processing
}

/*  loop0はフェーズ判定を主に行う  */
void loop0 (void* pvParameters){
    int64_t starttime;
    while(1){
        g_loop0counter++;
        TickType_t starttick = xTaskGetTickCount();
        //Serial.println("loop0 is working");
        switch (g_Phase)
        {
            // 待機(light sleep)
            case PHASE_WAIT:
            {
                Serial.println("After 5 seconds enter light sleep mode");
                delay(5000);
                esp_light_sleep_start();
                Serial.println("I woke up");
                break;
            }

            // テストフェーズ
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
                    // フライトモード移行条件の検証
                    case TEST_FLIGHTMODE:
                    {   
                        Serial.println("[TEST] Entry LAUNCH sequence [TEST]");
                        break;
                    }

                    // 離床検知条件の検証
                    case TEST_LAUNCH:
                    {
                        if(launchDecide(25000,5)){
                            Serial.println("[TEST] READY for launch [TEST]");
                        }else{
                            Serial.println("[TEST] NOT READY for launch [TEST]");
                        }
                        break;
                    }

                    // 翼展開機構動作指令条件の検証(高度)
                    case TEST_WINGALT:
                    {
                        if(wingaltDecide(5)){
                            Serial.println("[TEST] READY for expand the wing [TEST]");
                        }else{
                            Serial.println("[TEST] NOT READY for expand the wing [TEST]");
                        }
                        break;
                    }

                    // 翼展開機構動作指令条件の検証(タイマー)
                    case TEST_WINGTIMER:
                    {
                        if(g_loop0counter < 10){
                            Serial.println("[TEST] Start Verify the wing expansion (TIMER) [TEST]");
                            starttime = esp_timer_get_time();
                        }
                        Serial.println("ENTRY : TEST_WINGTIMER");

                        int64_t entrytime = esp_timer_get_time();
                        int64_t elapsedtime = entrytime - starttime;
                        Serial.print("elapsed time is ");
                        Serial.printf("% "PRId64" \n",elapsedtime);
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

                    // 何もしない
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

            // 発射判定
            case PHASE_LAUNCH:
            {
                if(launchDecide(30000,10)){
                    Serial.println("[FLIGHT] READY for launch [FLIGHT]");
                    starttime = esp_timer_get_time();
                }else{
                    Serial.println("[FLIGHT] NOT READY for launch [FLIGHT]");
                }
                break;   
            }

            // 離床検知
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
                /*　制御用サーボ動作
                ledcWrite(CHANNEL0, 1023);
                ledcWrite(CHANNEL1, 1023);
                */
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

        // コマンドを待つ
        if(!phaselock && COMM.available() > 0){
            g_receivedcommand = COMM.readStringUntil('\n');
            //Serial.print(g_receivedcommand);
            //Serial.println(" g_receivedcommand received.");
            g_command = g_receivedcommand[0];

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
        // ティック数計測
        TickType_t starttick = xTaskGetTickCount(); 
        Serial.print("fifoCount at first are ");
        Serial.printf("%"PRId16"\n",fifoCount);

        float altitude = bmp.readAltitude(1013.25);

        // MPU6050が使用不能の時、エマスト発動
        if (!dmpReady){
            Serial.println("!!! EMERGENCY EMERGENCY EMERGENCY !!!");
            g_Phase = PHASE_EMERGENCY;
        }

        // MPU6050の割り込みを待つ and FIFOに余裕があるかを確認
        while (!mpuInterrupt && fifoCount < packetSize) {
            if (mpuInterrupt && fifoCount < packetSize) {
            // 無限ループからの脱出を試みる
            fifoCount = mpu.getFIFOCount();
            }  
        }

        // 割り込みフラッグをリセットして、INT_STATUS バイトを取得する
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // 現在のFIFOカウントを確認
        fifoCount = mpu.getFIFOCount();

        // オーバーフローをチェック(while中にvTaskDelay等の遅れが大きすぎると発生する)
        if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
            // 計測を続けるためにFIFOをリセットする
            mpu.resetFIFO();
            fifoCount = mpu.getFIFOCount();
            Serial.println(F("FIFO overflow!"));
        } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
            // 利用可能なデータ長を取得(これは非常に短い時間で行われる)
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // FIFOからパケットを読み込む
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // 利用可能なパケットが1つ以上ある場合は、FIFOのカウントに数える
            // (これにより、割り込みを待たずにすぐに詳細を読むことができる)
            fifoCount -= packetSize;

            // 加速度ベクトルの大きさを取得
            mpu.dmpGetAccel(&aa, fifoBuffer);
            float magnitude = aa.getMagnitude();
            
            // 取得したデータをキューに送信
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
    if(testcommand == '0') return TEST_FLIGHTMODE;
    if(testcommand == '1') return TEST_LAUNCH;
    if(testcommand == '2') return TEST_WINGALT;
    if(testcommand == '3') return TEST_WINGTIMER;
    return oldTest;
}

bool launchDecide(int magnitudecriterion, int countercriterion){
    /*
    magnitudecriterion : 加速度ベクトルの大きさの基準
    countercriterion : 何回連続したときに判定するか
    */
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
        if((g_loop0counter - g_launchcounter) == 1){ //連続かどうかの判定
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
    /*
    countercriterion : 何回連続したときに判定するか
    */
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

    if(g_wingcounter < 5){ // 最初の5回分を埋める
        g_wingheight[g_wingcounter] = altitude;
        g_wingcounter++;
    }else{
        g_wingheight[0] = altitude;
    }

    // 移動平均を算出
    g_wingnewaverage = 0.0;
    for(int i = 0; i < validation; i++) g_wingnewaverage += g_wingheight[i];
    g_wingnewaverage = g_wingnewaverage / float(validation);
    Serial.print("Average is ");      
    Serial.println(g_wingnewaverage);          

    if(g_wingnewaverage < g_wingoldaverage){
        if((g_loop0counter - g_wingcounter) == 1){ // 連続かどうかの判定
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