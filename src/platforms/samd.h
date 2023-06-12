#include <Arduino.h>

// 24 bits timer
#include <TimerTCC0.h>
// uses TimerTcc0
// 16 bits timer
//#include <TimerTC3.h>
// uses TimerTc3
#define ATOMIC(X) noInterrupts(); X; interrupts();

IntervalTimer _uclockTimer;

// forward declaration of ISR
void uclockISR();

void initTimer(uint32_t init_clock)
{
    TimerTcc0.initialize(init_clock);

    // attach to generic uclock ISR
    TimerTcc0.attachInterrupt(uclockISR);
}

void setTimer(uint32_t us_interval)
{
    TimerTcc0.setPeriod(us_interval);
}