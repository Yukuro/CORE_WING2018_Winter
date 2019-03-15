#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

LoRaClass lora9216e5; //921.6MHz
LoRaClass lora9218e5; //921.8MHz
LoRaClass lora9220e5; //922.0MHz

const int EMGSERVO1PIN = 4;
const int EMGSERVO2PIN = 5;

const int FLIGHTMODEPIN = 13;
const int EMGMODEPIN = 14;
//const int MKRTOESPPIN = 6;

int g_loopcounter = 1;
int g_emgloopcounter = 1;
int g_oldsignal = -1;

Servo emgservo1;
Servo emgservo2;

enum loraFrequency{
  LORA9216E5,
  LORA9218E5,
  LORA9220E5,
  LORASTAND
} g_Freq;

bool g_commflag = true; //通信許可判別用フラッグ(許可=true);
bool g_emgflag = false; //エマスト判断用フラッグ(エマストと判断=true)

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
  emgservo1.attach(EMGSERVO1PIN);
  emgservo2.attach(EMGSERVO2PIN);

  emgservo1.write(95);
  delay(100);
  emgservo2.write(95);
  delay(100);

  pinMode(FLIGHTMODEPIN, OUTPUT); //フライトモード遷移用
  digitalWrite(FLIGHTMODEPIN, HIGH);
  pinMode(EMGMODEPIN, INPUT_PULLUP); //ESP32からのエマスト通知用

  //sleep回復用ピンの設定
  //pinMode(WAKEUP_PIN, OUTPUT);
  //digitalWrite(WAKEUP_PIN,LOW);
}

void loop() {
  String rcvcommand = receiveCommand();
  Serial.print("Received command is ");
  Serial.println(rcvcommand);

  //エマスト時動作
  int nowsignal = digitalRead(EMGMODEPIN);
  Serial.print("EMGMODEPIN is ");
  Serial.println(nowsignal);
  /* 連続判定 */
  int emgcounter;
  if(nowsignal != g_oldsignal) emgcounter = 0;
  if(digitalRead(EMGMODEPIN) == LOW){
    if((g_loopcounter - g_emgloopcounter) == 1) emgcounter++;
    g_emgloopcounter = g_loopcounter;
  }
  g_oldsignal = nowsignal;
  Serial.print("[EMG] emgcounter is ");
  Serial.println(emgcounter);
  if(emgcounter >= 5) g_emgflag = true;
  if(rcvcommand == "emg") g_emgflag = true;

  if(g_emgflag){
    Serial.println("[EMG] Emergency situation occurred !!! [EMG]");
    delay(100);
    emgservo1.write(160);
    delay(100);
    emgservo2.write(175);
    delay(100);
    while(1); //エマスト時には以後一切のコマンドを禁止(間違い防止)
  }

  //フライトモードに遷移
  if(rcvcommand == "l"){
    Serial.print("[FLIGHT] I received flight signal ... "); //TODO remove
    digitalWrite(FLIGHTMODEPIN, LOW);
    delay(100);
    Serial.println("done");
  }

  //操作のキャンセル
  if(rcvcommand == "cncl"){
    Serial.print("[FLIGHT] Cancel all operations ...");
    digitalWrite(FLIGHTMODEPIN, HIGH);
    delay(100);
    Serial.println(" done.");
  }
  g_loopcounter++;
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
