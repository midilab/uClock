#include <Arduino.h>
#include "pico/sync.h"

// todo: make this a build flag, so user can choose which method to use?
#define MULTICORE

#ifdef MULTICORE
    // use interrupt version -- works for 2 cores ie can run loop1() and loop() simultaneously as well as the clock callback?

    // RPi-specific timer
    struct repeating_timer timer;

    #define ATOMIC(X) { uint32_t __interrupt_mask = save_and_disable_interrupts(); X; restore_interrupts(__interrupt_mask); }

    // forward declaration of uClockHandler
    void uClockHandler();

    // ISR handler -- called when tick happens
    bool handlerISR(repeating_timer *timer)
    {
        uClockHandler();

        return true;
    }

    void initTimer(uint32_t init_clock) {
        // set up RPi interrupt timer
        // todo: actually should be -init_clock so that timer is set to start init_clock us after last tick, instead of init_clock us after finished processing last tick!
        add_repeating_timer_us(init_clock, &handlerISR, NULL, &timer);
    }

    void setTimer(uint32_t us_interval) {
        cancel_repeating_timer(&timer);
        // todo: actually should be -us_interval so that timer is set to start init_clock us after last tick, instead of init_clock us after finished processing last tick!
        add_repeating_timer_us(us_interval, &handlerISR, NULL, &timer);
    }
#else
    // use FreeRTOS scheduling/mutex version -- doesn't work (task starts but does not run) if using loop1() ie core 2

    #include "FreeRTOS.h"
    #include <task.h>
    #include <semphr.h>

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
        // todo: actually should be -init_clock so that timer is set to start init_clock us after last tick, instead of init_clock us after finished processing last tick!
        add_repeating_timer_us(init_clock, &handlerISR, NULL, &timer);
    }

    void setTimer(uint32_t us_interval) {
        cancel_repeating_timer(&timer);
        // todo: actually should be -us_interval so that timer is set to start init_clock us after last tick, instead of init_clock us after finished processing last tick!
        add_repeating_timer_us(us_interval, &handlerISR, NULL, &timer);
    }

#endif