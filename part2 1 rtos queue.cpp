// part 2 1 rtos queue
#include <Arduino_FreeRTOS.h>

// Task 1: Blink LED every 500 ms
void TaskBlink(void *pvParameters) {
  pinMode(13, OUTPUT);
  while (1) {
    digitalWrite(13, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(13, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task 2: Print to Serial every 1000 ms
void TaskPrint(void *pvParameters) {
  while (1) {
    Serial.println("Hello from FreeRTOS Task!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);
  xTaskCreate(TaskBlink, "Blink", 128, NULL, 1, NULL);
  xTaskCreate(TaskPrint, "Print", 128, NULL, 1, NULL);
}

void loop() {}
