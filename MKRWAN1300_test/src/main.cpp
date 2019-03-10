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
  String rcvcommand = receiveCommand();
  Serial.print("Received command is ");
  Serial.println(rcvcommand);

  //エマスト時動作
  if(rcvcommand == "e"){
    Serial.println("[EMG] Emergency situation occurred !!! [EMG]");
    delay(100);
    //servo1.write();
    //servo2.write();
    while(1); //エマスト時には以後一切のコマンドを禁止(間違い防止)
  }

  //sleep復帰
  if(rcvcommand == "x"){
    digitalWrite(WAKEUP_PIN,HIGH);
  }

  Serial.print("[DEBUG] g_commflag states is ");
  Serial.println(g_commflag);
  //UART1経由でコマンド転送
  if(g_commflag){
    Serial.println("[DEBUG] : Entry send sequence");
    Serial1.println(rcvcommand);
    Serial1.flush();
    Serial.println("[DEBUG] : SUCESSFUL send via UART1");
  }

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
        Serial.println(tmpcommand);
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
          Serial.print("[DEBUG] tmp command is ");
          Serial.println(tmpcommand);
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
          Serial.print("[DEBUG] tmp command is ");
          Serial.println(tmpcommand);
          if(tmpcommand != -1) command.concat(char(tmpcommand));
        }while(tmpcommand != -1);
      }
    }
  }

  return command;
}