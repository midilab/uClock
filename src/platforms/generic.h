#include <Arduino.h>

/* 
    Generic fallback approach that doesn't rely on any particular MCU's interrupts or RTOS threads etc.
    Simply checks micros() and compares last time tick happened and interval size to determine when a tick is due.
    requires calling uClockCheckTime(micros()); inside loop() in order to trigger tick processing. 
    function signature: void uClockCheckTime(uint32_t micros_time);
*/

#define ATOMIC(X) X;

// forward declaration of ISR
void uClockHandler();

uint32_t uclock_last_time_ticked;
uint32_t uclock_us_interval;

// call this as often as possible to tick the uClock
void uClockCheckTime(uint32_t micros_time) {
    if (micros_time - uclock_last_time_ticked >= uclock_us_interval) {
        uclock_last_time_ticked = micros_time;
        uClockHandler();
    }
}

void initTimer(uint32_t init_clock)
{
    // basically nothing to do for software-implemented version..?
    uclock_last_time_ticked = micros();
}

void setTimer(uint32_t us_interval)
{
    uclock_us_interval = us_interval;
}