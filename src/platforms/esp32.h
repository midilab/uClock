#include <Arduino.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TIMER_ID	0
hw_timer_t * _uclockTimer = NULL;
// mutex control for ISR
//portMUX_TYPE _uclockTimerMux = portMUX_INITIALIZER_UNLOCKED;
//#define ATOMIC(X) portENTER_CRITICAL_ISR(&_uclockTimerMux); X; portEXIT_CRITICAL_ISR(&_uclockTimerMux);

// FreeRTOS main clock task size in bytes
#define CLOCK_STACK_SIZE    2048
// semaphore to signal the task from the ISR
SemaphoreHandle_t _semaphore;
// mutex to protect the shared resource
SemaphoreHandle_t _mutex;
// mutex control for task
#define ATOMIC(X)     \
    do {              \
        xSemaphoreTake(_mutex, portMAX_DELAY); \
        X;            \
        xSemaphoreGive(_mutex); \
    } while (0);

// forward declaration of uClockHandler
void uClockHandler();

// ISR handler
void ARDUINO_ISR_ATTR handlerISR(void)
{
    // notify the clockTask using the semaphore
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_semaphore, &xHigherPriorityTaskWoken);

    // context switch is required? request it
    if (xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// task for user clock process
void clockTask(void *pvParameters)
{
    while (1)
        if (xSemaphoreTake(_semaphore, portMAX_DELAY) == pdTRUE)
            uClockHandler();
}

void initTimer(uint32_t init_clock)
{
    _uclockTimer = timerBegin(TIMER_ID, 80, true);

    // attach to generic uclock ISR
    timerAttachInterrupt(_uclockTimer, &handlerISR, false);

    // init clock tick time
    timerAlarmWrite(_uclockTimer, init_clock, true); 

    // activate it!
    timerAlarmEnable(_uclockTimer);

    // initialize the semaphore
    _semaphore = xSemaphoreCreateBinary();

    // initialize the mutex for shared resource access
    _mutex = xSemaphoreCreateMutex();

    // create the clockTask
    xTaskCreate(clockTask, "clockTask", CLOCK_STACK_SIZE, NULL, 1, NULL);
}

void setTimer(uint32_t us_interval)
{
    timerAlarmWrite(_uclockTimer, us_interval, true); 
}