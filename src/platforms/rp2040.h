#include <Arduino.h>
#include "pico/sync.h"

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
    if (!add_repeating_timer_us(-static_cast<int64_t>(init_clock ? init_clock : 1), &handlerISR, NULL, &timer)) {
        timer.alarm_id = 0;
        panic("uClock: unable to allocate RP2040 timer");
    }
}

void setTimer(uint32_t us_interval) {
    ATOMIC(timer.delay_us = -static_cast<int64_t>(us_interval ? us_interval : 1))
}
