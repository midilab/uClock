#include <Arduino.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// esp32-specific timer
#define TIMER_ID	0
hw_timer_t * _uclockTimer = NULL;
// mutex control for ISR
//portMUX_TYPE _uclockTimerMux = portMUX_INITIALIZER_UNLOCKED;
//#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uclockTimerMux); X; portEXIT_CRITICAL_ISR(&_uclockTimerMux);

// FreeRTOS main clock task size in bytes
#define CLOCK_STACK_SIZE    5*1024 // adjust for your needs, a sequencer with heavy serial handling should be large in size
TaskHandle_t taskHandle;
// mutex to protect the shared resource
SemaphoreHandle_t _mutex;
// mutex control for task
#define ATOMIC(X) xSemaphoreTake(_mutex, portMAX_DELAY); X; xSemaphoreGive(_mutex);

// forward declaration of uClockHandler
void uClockHandler();

// ISR handler
void ARDUINO_ISR_ATTR handlerISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Send a notification to task1
    vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

    _uclockTimer = timerBegin(TIMER_ID, 80, true);

    // attach to generic uclock ISR
    timerAttachInterrupt(_uclockTimer, &handlerISR, false);

    // init clock tick time
    timerAlarmWrite(_uclockTimer, init_clock, true); 

    // activate it!
    timerAlarmEnable(_uclockTimer);
}

void setTimer(uint32_t us_interval)
{
    timerAlarmWrite(_uclockTimer, us_interval, true); 
}