#include <Arduino_FreeRTOS.h>
#include <queue.h>

// Make IRAM_ATTR a no-op on non-ESP32 to keep code portable
#ifndef ARDUINO_ARCH_ESP32
  #define IRAM_ATTR
#endif

// Cross-port "yield from ISR" helper:
// - AVR (Arduino_FreeRTOS): call taskYIELD() only if a higher-priority task was woken
// - Others (e.g., ESP32): use the standard macro with the argument
#if defined(ARDUINO_ARCH_AVR)
  #define YIELD_FROM_ISR_IF_NEEDED(hptw) do { if ((hptw) == pdTRUE) { taskYIELD(); } } while (0)
#else
  #define YIELD_FROM_ISR_IF_NEEDED(hptw) portYIELD_FROM_ISR(hptw)
#endif

QueueHandle_t buttonQueue;

void IRAM_ATTR buttonISR() {
  const int signal = 1;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Send from ISR; request a context switch if this unblocks a higher-priority task
  xQueueSendFromISR(buttonQueue, &signal, &xHigherPriorityTaskWoken);
  YIELD_FROM_ISR_IF_NEEDED(xHigherPriorityTaskWoken);
}

void TaskLED(void *pvParameters) {
  pinMode(13, OUTPUT);
  int msg;
  for (;;) {
    if (xQueueReceive(buttonQueue, &msg, portMAX_DELAY) == pdTRUE) {
      digitalWrite(13, !digitalRead(13));  // toggle LED on each button press
    }
  }
}

void setup() {
  pinMode(2, INPUT_PULLUP);  // use internal pull-up; button to GND
  buttonQueue = xQueueCreate(5, sizeof(int));

  // Attach ISR on falling edge (button press)
  attachInterrupt(digitalPinToInterrupt(2), buttonISR, FALLING);

  // Create LED task (increase stack if you add more logic)
  xTaskCreate(TaskLED, "LED", 128, NULL, 2, NULL);
}

void loop() {
  // Empty: FreeRTOS takes over after setup()
}

