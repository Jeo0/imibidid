// part 2 rtos precise timing

#include <Arduino_FreeRTOS.h>

// QueueHandle_t displayQueue;

void TaskBlinkPrecise(void *pvParameters) {
  pinMode(13, OUTPUT);
  const TickType_t period = 500 / portTICK_PERIOD_MS;
  TickType_t lastWake = xTaskGetTickCount();

//   int readValue =0 ;

  while (1) {
    // readValue = digitalRead(13);
    // xQueueSend(displayQueue, &readValue, portMAX_DELAY);

    digitalWrite(13, !digitalRead(13));
    vTaskDelayUntil(&lastWake, period);  // exact cadence
  }
}

// void TaskPrintDisplay(void *pvParameters){
//     // int WHAT;
//     while(1){
//         if(xQueueReceive(displayQueue, &WHAT, portMAX_DELAY)){
//             Serial.println("noice");
//             Serial.println(WHAT);
//         }
//     }
// }

void setup() {
  xTaskCreate(TaskBlinkPrecise, "BlinkPrecise", 128, NULL, 1, NULL);
//   xTaskCreate(TaskPrintDisplay, "printDisplay", 128, NULL, 1, NULL);
}
