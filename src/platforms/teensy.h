#include <Arduino.h>

#define ATOMIC(X) noInterrupts(); X; interrupts();

IntervalTimer _uclockTimer;

// forward declaration of ISR
void uClockHandler();

void initTimer(uint32_t init_clock)
{
    _uclockTimer.begin(uClockHandler, init_clock); 

    // Set the interrupt priority level, controlling which other interrupts
    // this timer is allowed to interrupt. Lower numbers are higher priority, 
    // with 0 the highest and 255 the lowest. Most other interrupts default to 128. 
    // As a general guideline, interrupt routines that run longer should be given 
    // lower priority (higher numerical values).
    _uclockTimer.priority(80);
}

void setTimer(uint32_t us_interval)
{
    _uclockTimer.update(us_interval);
}