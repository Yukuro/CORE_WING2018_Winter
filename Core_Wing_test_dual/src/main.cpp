#include <Arduino.h>

portTickType Delay1000 = 1000 / portTICK_PERIOD_MS;
TaskHandle_t th[2];

void task0(void* pvParameters){
  while(1){
    Serial.println("This is task0");
    Serial.println(xPortGetCoreID());
    vTaskDelay(Delay1000);
  }
}

void task1(void* pvParameters){
  while(1){
    Serial.println("This is task1");
    Serial.println(xPortGetCoreID());
    vTaskDelay(Delay1000);
  }
}

void setup() {
  Serial.begin(9600);
  xTaskCreatePinnedToCore(task0, "Task0", 8192, NULL, 2, &th[0], 0);
  delay(500);
  xTaskCreatePinnedToCore(task1, "Task1", 8192, NULL, 2, &th[1], 1);
  delay(500);
}

void loop() {
}