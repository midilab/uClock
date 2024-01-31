#include <Arduino.h>
#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>
#include "pico/sync.h"

// RPi-specific timer
struct repeating_timer timer;

// FreeRTOS main clock task size in bytes
#define CLOCK_STACK_SIZE    5*1024 // adjust for your needs, a sequencer with heavy serial handling should be large in size
TaskHandle_t taskHandle;
// mutex to protect the shared resource
SemaphoreHandle_t _mutex;
// mutex control for task
#define ATOMIC(X) xSemaphoreTake(_mutex, portMAX_DELAY); X; xSemaphoreGive(_mutex);

// forward declaration of uClockHandler
void uClockHandler();

// ISR handler -- called when tick happens
bool handlerISR(repeating_timer *timer)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Send a notification to task1
    vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return true;
}

// task for user clock process
void clockTask(void *pvParameters)
{
    while (1) {
        // wait for a notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uClockHandler();
    }
}

void initTimer(uint32_t init_clock)
{
    // initialize the mutex for shared resource access
    _mutex = xSemaphoreCreateMutex();

    // create the clockTask
    xTaskCreate(clockTask, "clockTask", CLOCK_STACK_SIZE, NULL, 1, &taskHandle);

    // set up RPi interrupt timer
    add_repeating_timer_us(init_clock, &handlerISR, NULL, &timer);
}

void setTimer(uint32_t us_interval) {
    cancel_repeating_timer(&timer);
	add_repeating_timer_us(us_interval, &handlerISR, NULL, &timer);
}