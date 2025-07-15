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
    // todo: actually should be -init_clock so that timer is set to start init_clock us after last tick, instead of init_clock us after finished processing last tick!
    add_repeating_timer_us(init_clock, &handlerISR, NULL, &timer);
}

void setTimer(uint32_t us_interval) {
    cancel_repeating_timer(&timer);
    // todo: actually should be -us_interval so that timer is set to start init_clock us after last tick, instead of init_clock us after finished processing last tick!
    add_repeating_timer_us(us_interval, &handlerISR, NULL, &timer);
}