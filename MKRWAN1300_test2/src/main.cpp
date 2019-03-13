/*
SENDER(地上局)
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

LoRaClass lora9216e5; //921.6MHz
LoRaClass lora9218e5; //921.8MHz
LoRaClass lora9220e5; //922.0MHz

const int DEBUGLED_PIN = 5;

enum loraFrequency{
  LORA9216E5,
  LORA9218E5,
  LORA9220E5,
  LORASTAND
} g_Freq;

bool g_commflag = true; //通信許可判別用フラッグ(許可=true);

void blinkLED(const int LEDPIN, int time);
loraFrequency carrierSense();
bool receiveData();
bool checkSignal(const String command);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000);
  Serial1.begin(115200);

  pinMode(DEBUGLED_PIN,OUTPUT);
  digitalWrite(DEBUGLED_PIN, LOW);

  while (!Serial);
  if(!lora9216e5.begin(9216E5)){
    while(1) blinkLED(DEBUGLED_PIN, 100);
  }
  if(!lora9218e5.begin(9218E5)){
    blinkLED(DEBUGLED_PIN, 100);
    while(1) blinkLED(DEBUGLED_PIN, 100);
  }
  if(!lora9220e5.begin(922E6)){
    blinkLED(DEBUGLED_PIN, 100);
    while(1) blinkLED(DEBUGLED_PIN, 100);
  }
  /* 出力を13dBmに */
  lora9216e5.setTxPower(13);
  lora9218e5.setTxPower(13);
  lora9220e5.setTxPower(13);

  //注意書き
  Serial.println("DO NOT USE ENTER KEY");
}

void loop() {
  bool rcvmasterflag = false; //マスター信号受信通知用フラッグ(信号受信=true);

  /* コマンド送信処理 */
  //Serial.println("[DEBUG] loop start");
  Serial.println("[DEBUG] Start send sequence");
  g_commflag = true; //デフォルトでは送信許可

  g_Freq = carrierSense();
  if(g_Freq == LORASTAND) g_commflag = false; //キャリアセンス失敗時は送信禁止
  switch(g_Freq){
    case LORA9216E5:
    {
      Serial.println("[INPUT:1s] command at 921.6MHz : ");
      String sendcommand = Serial.readStringUntil('\n'); //シリアルバッファから文字列受け取り
      Serial.print("[DEBUG] command is "); 
      Serial.print(sendcommand);
      Serial.print(" -> ");

      if(g_commflag){
        //sendcommand = "x2G&5," + sendcommand; //送信識別子をつける
        Serial.print(sendcommand);

        unsigned long starttime = micros(); //送信時間計測開始
        lora9216e5.beginPacket();
        lora9216e5.print(sendcommand);
        Serial.println(" sended");
        lora9216e5.endPacket();
        unsigned long endtime = micros(); //送信時間計測終了

        Serial.print("[DEBUG] Sending time is ");
        Serial.println(endtime - starttime);
      }else{
        Serial.println("[NOTICE] sending failed ...");
      }
      break;
    }
    
    case LORA9218E5:
    {
      Serial.println("[INPUT:2s] command at 921.8MHz : ");
      String sendcommand = Serial.readStringUntil('\n');
      if(sendcommand.length() > 0){
        Serial.print("[DEBUG] command is "); Serial.print(sendcommand); Serial.print("... ");

        if(g_commflag){
          //sendcommand = "x2G&5," + sendcommand; //送信識別子をつける

          unsigned long starttime = micros(); //送信時間計測開始
          Serial.println(sendcommand);
          lora9216e5.beginPacket();
          lora9216e5.print(sendcommand);
          Serial.println("sended");
          lora9216e5.endPacket();
          unsigned long endtime = micros(); //送信時間計測終了

          Serial.print("[DEBUG] Sending time is ");
          Serial.println(endtime - starttime);
        }else{
          Serial.println("[NOTICE] sending failed ...");
        }
      }
      break;
    }

    case LORA9220E5:
    {
      Serial.println("[INPUT:2s] command at 922.0MHz : ");
      String sendcommand = Serial.readStringUntil('\n');
      if(sendcommand.length() > 0){
        Serial.print("[DEBUG] command is "); Serial.print(sendcommand); Serial.print("... ");

        if(g_commflag){
          //sendcommand = "x2G&5," + sendcommand; //マスター側識別子をつける

          unsigned long starttime = micros(); //送信時間計測開始
          Serial.println(sendcommand);
          lora9216e5.beginPacket();
          lora9216e5.print(sendcommand);
          Serial.println("sended");
          lora9216e5.endPacket();
          unsigned long endtime = micros(); //送信時間計測終了

          Serial.print("[DEBUG] Sending time is ");
          Serial.println(endtime - starttime);
        }else{
          Serial.println("[NOTICE] sending failed ...");
        }
      }
      break;
    }

    default:
    {
      break;
    }
  }

  delay(52);
}

void blinkLED(const int LEDPIN, int time){
  digitalWrite(LEDPIN, HIGH);
  delay(time);
  digitalWrite(LEDPIN, LOW);
  delay(time);
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

bool receiveData(){
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
        Serial.print("[DEBUG] tmp command is ");
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

  Serial.print("[DEBUG:receiveData] command is ");
  Serial.println(command);

  if(Serial1.print(command) > 0){
    return true;
  }else{
    return false;
  }
}

bool checkSignal(const String command){
  const String identificate = "d+A84";
  //Serial.print("[DEBUG] command's length is ");
  //Serial.println(command.length());
  int indexsignal = command.indexOf(identificate);
  //Serial.print("[DEBUG] index signal is ");
  //Serial.println(indexsignal);
  if(indexsignal >= 0) return true;
  return false;
}