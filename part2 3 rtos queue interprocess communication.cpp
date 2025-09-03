// part2 3 rtos queues producer timer 
// using queue for safe inter process communication   
// 1 task reads potentionmeter and send data
// 1 task receives data and display

#include <Arduino_FreeRTOS.h>
#include <queue.h>

QueueHandle_t sensorQueue;

void TaskSensor(void *pvParameters) {
  int value;
  while (1) {
    value = analogRead(A0);                                // 0..1023
    xQueueSend(sensorQueue, &value, portMAX_DELAY);        // enqueue
    vTaskDelay(200 / portTICK_PERIOD_MS);                  // 5 H
  }
}

void TaskDisplay(void *pvParameters) {
  int received;
  while (1) {
    if (xQueueReceive(sensorQueue, &received, portMAX_DELAY)) {
      Serial.print("Potentiometer: ");
      Serial.println(received);
    }
  }
}

void setup() {
  Serial.begin(9600);
  sensorQueue = xQueueCreate(5, sizeof(int));
  xTaskCreate(TaskSensor,  "Sensor", 128, NULL, 1, NULL);
  xTaskCreate(TaskDisplay, "Display", 128, NULL, 1, NULL);
}

void loop() {}
