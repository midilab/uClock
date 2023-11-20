#include <Arduino.h>

// 24 bits timer
#include <TimerTCC0.h>
// uses TimerTcc0
// 16 bits timer
//#include <TimerTC3.h>
// uses TimerTc3
#define ATOMIC(X) noInterrupts(); X; interrupts();

// forward declaration of ISR
void uClockHandler();

void initTimer(uint32_t init_clock)
{
    TimerTcc0.initialize(init_clock);

    // attach to generic uclock ISR
    TimerTcc0.attachInterrupt(uClockHandler);
}

void setTimer(uint32_t us_interval)
{
    TimerTcc0.setPeriod(us_interval);
}