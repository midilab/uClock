#include <Arduino.h>

//#define ATOMIC(X) noInterrupts(); X; interrupts();
#define ATOMIC(X) X;

// forward declaration of ISR
void uClockHandler();

uint32_t uclock_last_time_ticked;
uint32_t uclock_us_interval;

// call this as often as possible to tick the uClock
void uClockCheckTime(uint32_t micros_time) {
    if (micros_time - uclock_last_time_ticked >= uclock_us_interval) {
        uclock_last_time_ticked = micros();
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
    Serial.printf("setTimer(%d)\n", us_interval); Serial.flush();
}