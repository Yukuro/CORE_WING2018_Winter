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
  while (!Serial);
  if(!lora9216e5.begin(9216E5)){
    Serial.println("SETUP : 921.6MHz ...");
    while(1);
  }
  if(!lora9218e5.begin(9218E5)){
    Serial.println("SETUP : 921.8MHz ...");
    while(1);
  }
  if(!lora9220e5.begin(922E6)){
    Serial.println("SETUP : 922.0MHz ...");
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
  digitalWrite(WAKEUP_PIN,HIGH);
}

void loop() {
  String rcvcommand = receiveCommand();

  //エマスト時動作
  if(rcvcommand == "e"){
    //servo1.write();
    //servo2.write();
    while(1); //エマスト時には以後一切のコマンドを禁止(間違い防止)
  }

  //sleep復帰
  if(rcvcommand == "x"){
    digitalWrite(WAKEUP_PIN,LOW);
  }

  //UART1経由でコマンド転送
  Serial1.print(rcvcommand);
  Serial1.print('\n');
}

String receiveCommand(){
  int packetSize;
  String command;

  packetSize = lora9216e5.parsePacket();
  while(lora9216e5.available()){
    if(packetSize){
      command = String(lora9216e5.read());
    }
  }
  packetSize = lora9218e5.parsePacket();
  if(packetSize){
    while(lora9218e5.available()){
      if(packetSize){
        command = String(lora9218e5.read());
      }
    }
  }
  packetSize = lora9220e5.parsePacket();
  if(packetSize){
    while(lora9220e5.available()){
      if(packetSize){
        command = String(lora9220e5.read());
      }
    }
  }

  return command;
}