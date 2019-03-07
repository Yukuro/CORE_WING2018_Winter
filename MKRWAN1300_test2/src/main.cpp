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

void setup() {
  Serial.println("[DEBUG] setup start");
  pinMode(DEBUGLED_PIN,OUTPUT);
  digitalWrite(DEBUGLED_PIN, LOW);
  
  Serial.begin(115200);
  Serial.setTimeout(3000);
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
}

void loop() {
  Serial.println("[DEBUG] loop start");

  g_Freq = carrierSense();
  if(g_Freq != LORASTAND) g_commflag = false; //キャリアセンス失敗時は送信禁止
  switch(g_Freq){
    case LORA9216E5:
    {
      String sendcommand;
      if(Serial.available() > 0){
        sendcommand = Serial.readStringUntil('\n');
        if(sendcommand.indexOf("[DEBUG]") != -1) g_commflag = false;
      }
      if(g_commflag){
        lora9216e5.beginPacket();
        lora9216e5.print(sendcommand);
        lora9216e5.print('\n');
        lora9216e5.endPacket();
      }
      break;
    }
    
    case LORA9218E5:
    {
      String sendcommand;
      if(Serial.available() > 0){
        sendcommand = Serial.readStringUntil('\n');
        if(sendcommand.indexOf("[DEBUG]") != -1) g_commflag = false;
      }
      if(g_commflag){
      lora9218e5.beginPacket();
      lora9218e5.print(sendcommand);
      lora9218e5.print('\n');
      lora9218e5.endPacket();
      }
      break;
    }

    case LORA9220E5:
    {
      String sendcommand;
      if(Serial.available() > 0){
        sendcommand = Serial.readStringUntil('\n');
        if(sendcommand.indexOf("[DEBUG]") != -1) g_commflag = false;
      }
      if(g_commflag){
        lora9220e5.beginPacket();
        lora9220e5.print(sendcommand);
        lora9220e5.print('\n');
        lora9220e5.endPacket();
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
      int rssi = lora9216e5.packetRssi(); //RSSIを取得
      if(rssi <= -80) csallow = true; //-80dBm以上の出力があった場合には電波の放出を禁止
    }
    entrytime = micros();
  }
  if(csallow) return LORA9216E5;

  starttime = micros();
  entrytime = micros();
  while(entrytime - starttime <= 128){
  packetSize = lora9218e5.parsePacket();
    if(packetSize){
      int rssi = lora9218e5.packetRssi();
      if(rssi <= -80) csallow = true;
    }
    entrytime = micros();
  }
  if(csallow) return LORA9218E5;

  starttime = micros();
  entrytime = micros();
  while(entrytime - starttime <= 128){
  packetSize = lora9220e5.parsePacket();
    if(packetSize){
      int rssi = lora9220e5.packetRssi();
      if(rssi <= -80) csallow = true;
    }
    entrytime = micros();
  }
  if(csallow) return LORA9220E5;

  return LORASTAND;
}