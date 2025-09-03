#include <Arduino_FreeRTOS.h>
#include <queue.h>

// Keep code portable: IRAM_ATTR for ESP32; no-op on AVR
#ifndef ARDUINO_ARCH_ESP32
  #define IRAM_ATTR
#endif

// Cross-port "yield from ISR":
#if defined(ARDUINO_ARCH_AVR)
  // Arduino_FreeRTOS AVR: macro takes no args — call taskYIELD() if needed
  #define YIELD_FROM_ISR_IF_NEEDED(hptw) do { if ((hptw) == pdTRUE) { taskYIELD(); } } while (0)
#else
  // ESP32/others: standard macro with argument
  #define YIELD_FROM_ISR_IF_NEEDED(hptw) portYIELD_FROM_ISR(hptw)
#endif

QueueHandle_t sensorQueue;
QueueHandle_t buttonQueue;

volatile int ledDelayMs = 1000;  // default 1 Hz

// ---------- ISR: Button ----------
void IRAM_ATTR buttonISR() {
  const int signal = 1;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xQueueSendFromISR(buttonQueue, &signal, &xHigherPriorityTaskWoken);

  // Request context switch if a higher-priority task was unblocked
  YIELD_FROM_ISR_IF_NEEDED(xHigherPriorityTaskWoken);
}

// ---------- Task: LED Blinker ----------
void TaskLED(void *pvParameters) {
  pinMode(13, OUTPUT);
  int msg;
  for (;;) {
    digitalWrite(13, HIGH);
    vTaskDelay(ledDelayMs / portTICK_PERIOD_MS);
    digitalWrite(13, LOW);
    vTaskDelay(ledDelayMs / portTICK_PERIOD_MS);

    // Non-blocking check for button message
    if (xQueueReceive(buttonQueue, &msg, 0) == pdTRUE) {
      ledDelayMs = (ledDelayMs == 1000) ? 500 : 1000; // toggle 1s <-> 0.5s
    }
  }
}

// ---------- Task: Sensor Producer ----------
void TaskSensor(void *pvParameters) {
  int value;
  for (;;) {
    value = analogRead(A0);                          // 0..1023
    xQueueSend(sensorQueue, &value, portMAX_DELAY);  // enqueue
    vTaskDelay(200 / portTICK_PERIOD_MS);            // 5 Hz sample
  }
}

// ---------- Task: Serial Consumer ----------
void TaskDisplay(void *pvParameters) {
  int received;
  for (;;) {
    if (xQueueReceive(sensorQueue, &received, portMAX_DELAY) == pdTRUE) {
      Serial.print("Potentiometer: ");
      Serial.println(received);
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Queues
  sensorQueue = xQueueCreate(8, sizeof(int));
  buttonQueue = xQueueCreate(4, sizeof(int));

  // Button (to GND) with internal pull-up
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), buttonISR, FALLING);

  // Tasks (note: AVR stack size is in words for Arduino_FreeRTOS)
  xTaskCreate(TaskLED,     "LED",     128, NULL, 2, NULL);
  xTaskCreate(TaskSensor,  "Sensor",  128, NULL, 1, NULL);
  xTaskCreate(TaskDisplay, "Display", 160, NULL, 1, NULL); // extra for Serial
}

void loop() {}
