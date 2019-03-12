/*
RECIVER
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

LoRaClass lora9216e5; //921.6MHz
LoRaClass lora9218e5; //921.8MHz
LoRaClass lora9220e5; //922.0MHz

const int SERVO1_PIN = 4;
const int SERVO2_PIN = 5;
const int WAKEUP_PIN = 6;

Servo servo1;
Servo servo2;

enum loraFrequency{
  LORA9216E5,
  LORA9218E5,
  LORA9220E5,
  LORASTAND
} g_Freq;

bool g_commflag = true; //通信許可判別用フラッグ(許可=true);

String receiveCommand();
loraFrequency carrierSense();
bool checkSignal(const String command);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial);
  if(!lora9216e5.begin(9216E5)){
    Serial.println("FAILED: setup 921.6MHz ...");
    while(1);
  }
  if(!lora9218e5.begin(9218E5)){
    Serial.println("FAILED : setup 921.8MHz ...");
    while(1);
  }
  if(!lora9220e5.begin(922E6)){
    Serial.println("FAILED : setup 922.0MHz ...");
    while(1);
  }
  /* 出力を13dBmに */
  lora9216e5.setTxPower(13);
  lora9218e5.setTxPower(13);
  lora9220e5.setTxPower(13);

  //サーボの設定
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  //sleep回復用ピンの設定
  pinMode(WAKEUP_PIN, OUTPUT);
  digitalWrite(WAKEUP_PIN,LOW);
}

void loop() {
  Serial.println("[DEBUG] Entry receive sequence");
  bool rcvmasterflag = false; //マスター信号受信通知用フラッグ(信号受信=true);
  String rcvcommand = receiveCommand();

  Serial.print("Received command is ");
  Serial.println(rcvcommand);

  if(checkSignal(rcvcommand)){
    Serial.println("[DEBUG] I received master's command");
    rcvmasterflag = true;
    rcvcommand.remove(0,6); //コマンドを抽出(識別子を削除)
  }

  //TODO コマンド動作もrcvmasterflagを通じて動作させる

  //エマスト時動作
  if(rcvcommand == "e"){
    Serial.println("[EMG] Emergency situation occurred !!! [EMG]");
    delay(100);
    //servo1.write();
    //servo2.write();
    //while(1); //エマスト時には以後一切のコマンドを禁止(間違い防止)
  }

  //sleep復帰
  if(rcvcommand == "x"){
    digitalWrite(WAKEUP_PIN,HIGH);
  }

  Serial.print("[DEBUG] g_commflag states is ");
  Serial.println(g_commflag);
  //UART1経由でコマンド転送
  if(g_commflag){
    //Serial.println("[DEBUG] : Entry send UART1 sequence");
    Serial1.println(rcvcommand);
    Serial1.flush();
    Serial.println("[DEBUG] : SUCESSFUL send via UART1");
  }

  Serial.println("[DEBUG] Entry send sequence");
  //if(rcvmasterflag){
    /* センサ値データ送信処理 
    String sendcommand = "d+A84," + Serial1.readStringUntil('\n');
    g_commflag = true; //デフォルトでは送信許可

    g_Freq = carrierSense();
    if(g_Freq == LORASTAND) g_commflag = false; //キャリアセンス失敗時は送信禁止
    switch(g_Freq){
      case LORA9216E5:
      {
          Serial.print("[DEBUG] sendcommand is "); 
          Serial.print(sendcommand);
          Serial.print(" ... ");

          if(g_commflag){
            sendcommand = "d+A84," + sendcommand; //スレーブ側識別子をつける
            unsigned long starttime = micros();
            lora9216e5.beginPacket();
            lora9216e5.endPacket();
            Serial.println("sended");
            unsigned long endtime = micros();
            Serial.print("[DEBUG] Sending time is ");
            Serial.println(endtime - starttime);
          }
        break;
      }
      
      case LORA9218E5:
      {
        String sendcommand = Serial.readStringUntil('\n');
        if(sendcommand.length() > 0){
          Serial.print("[DEBUG] command is "); Serial.print(sendcommand); Serial.print("... ");

          if(g_commflag){
            unsigned long starttime = micros();
            lora9216e5.beginPacket();
            lora9216e5.print(sendcommand);
            lora9216e5.endPacket();
            Serial.println("sended");
            unsigned long endtime = micros();
            Serial.print("[DEBUG] Sending time is ");
            Serial.println(endtime - starttime);
          }
        }
        break;
      }

      case LORA9220E5:
      {
        String sendcommand = Serial.readStringUntil('\n');
        if(sendcommand.length() > 0){
          Serial.print("[DEBUG] command is "); Serial.print(sendcommand); Serial.print("... ");

          if(g_commflag){
            unsigned long starttime = micros();
            lora9216e5.beginPacket();
            lora9216e5.print(sendcommand);
            lora9216e5.endPacket();
            Serial.println("sended");
            unsigned long endtime = micros();
            Serial.print("[DEBUG] Sending time is ");
            Serial.println(endtime - starttime);
          }
        }
        break;
      }

      default:
      {
        break;
      }
    }
    */
  //}

  delay(50);
}

String receiveCommand(){
  int packetSize;
  String command = "";

  packetSize = lora9216e5.parsePacket();
  Serial.print("[DEBUG] packetSize is ");
  Serial.println(packetSize);
  while(lora9216e5.available() > 0){
    if(packetSize){
      int tmpcommand;
      do{
        tmpcommand = lora9216e5.read();
        //Serial.println(tmpcommand);
        if(tmpcommand != -1) command.concat(char(tmpcommand));
      }while(tmpcommand != -1);
    }
  }
  packetSize = lora9218e5.parsePacket();
  if(packetSize){
    while(lora9218e5.available()){
      if(packetSize){
        int tmpcommand;
        do{
          tmpcommand = lora9216e5.read();
          //Serial.print("[DEBUG] tmp command is ");
          //Serial.println(tmpcommand);
          if(tmpcommand != -1) command.concat(char(tmpcommand));
        }while(tmpcommand != -1);
      }
    }
  }
  packetSize = lora9220e5.parsePacket();
  if(packetSize){
    while(lora9220e5.available()){
      if(packetSize){
      int tmpcommand;
        do{
          tmpcommand = lora9216e5.read();
          //Serial.print("[DEBUG] tmp command is ");
          //Serial.println(tmpcommand);
          if(tmpcommand != -1) command.concat(char(tmpcommand));
        }while(tmpcommand != -1);
      }
    }
  }

  return command;
}

loraFrequency carrierSense(){
  /*
    ARIB STD-T108準拠
    キャリアセンス時間:128us
    キャリアセンスレベル:-80dBm以上
  */

  //経過時間計測
  unsigned long starttime = micros();
  unsigned long entrytime = micros();
  bool csallow = false; //電波放出許可判定(許可=true)
  int packetSize;

  while(entrytime - starttime <= 128){
    packetSize = lora9216e5.parsePacket();
      if(packetSize){
        Serial.print("[NOTICE] : Entry CS(921.6MHz) at RSSI = ");
        int rssi = lora9216e5.packetRssi(); //RSSIを取得
        Serial.println(rssi);
        if(rssi <= -80) csallow = true; //-80dBm以下の出力なら電波の放出を許可
      }else{
        csallow = true; //その周波数帯が使われていないなら電波の放出を許可
      }
      entrytime = micros();
  }
  if(csallow) return LORA9216E5;

  starttime = micros();
  entrytime = micros();
  while(entrytime - starttime <= 128){
    packetSize = lora9218e5.parsePacket();
      if(packetSize){
        Serial.print("[NOTICE] : Entry CS(921.8MHz) at RSSI = ");
        int rssi = lora9218e5.packetRssi();
        Serial.println(rssi);
        if(rssi <= -80) csallow = true;
      }else{
        csallow = true;
      }
      entrytime = micros();
  }
  if(csallow) return LORA9218E5;

  starttime = micros();
  entrytime = micros();
  while(entrytime - starttime <= 128){
    packetSize = lora9220e5.parsePacket();
      if(packetSize){
        Serial.print("[NOTICE] : Entry CS(922.0MHz) at RSSI = ");
        int rssi = lora9220e5.packetRssi();
        Serial.println(rssi);
        if(rssi <= -80) csallow = true;
      }else{
        csallow = true;
      }
      entrytime = micros();
  }
  if(csallow) return LORA9220E5;

  return LORASTAND;
}

bool checkSignal(const String command){
  const String identificate = "x2G&5";
  //Serial.print("[DEBUG] command's length is ");
  //Serial.println(command.length());
  int indexsignal = command.indexOf(identificate);
  //Serial.print("[DEBUG] index signal is ");
  //Serial.println(indexsignal);
  if(indexsignal >= 0) return true;
  return false;
}