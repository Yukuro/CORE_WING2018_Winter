#include <Arduino.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
HardwareSerial GPS(2);

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  GPS.begin(9600);
}

void loop() {
  while (GPS.available() > 0) {
    char c = GPS.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
      Serial.print("HOUR: "); Serial.println(gps.time.second(), 9);
    }
  }
}