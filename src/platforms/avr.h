#include <Arduino.h>

#define ATOMIC(X) noInterrupts(); X; interrupts();

// want a different avr clock support?
// TODO: we should do this using macro guards for avrs different clocks freqeuncy setup at compile time
#define AVR_CLOCK_FREQ	16000000

void initTimer(uint32_t init_clock)
{
    ATOMIC(
        // 16bits Timer1 init
        // begin at 120bpm (48.0007680122882 Hz)
        TCCR1A = 0; // set entire TCCR1A register to 0
        TCCR1B = 0; // same for TCCR1B
        TCNT1  = 0; // initialize counter value to 0
        // set compare match register for 48.0007680122882 Hz increments
        OCR1A = 41665; // = 16000000 / (8 * 48.0007680122882) - 1 (must be <65536)
        // turn on CTC mode
        TCCR1B |= (1 << WGM12);
        // Set CS12, CS11 and CS10 bits for 8 prescaler
        TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
        // enable timer compare interrupt
        TIMSK1 |= (1 << OCIE1A);
    )
}

void setTimer(uint32_t us_interval)
{
    float tick_hertz_interval = 1/((float)us_interval/1000000);

    uint32_t ocr;
    uint8_t tccr = 0;

    // 16bits avr timer setup
    if ((ocr = AVR_CLOCK_FREQ / ( tick_hertz_interval * 1 )) < 65535) {
        // Set CS12, CS11 and CS10 bits for 1 prescaler
        tccr |= (0 << CS12) | (0 << CS11) | (1 << CS10);
    } else if ((ocr = AVR_CLOCK_FREQ / ( tick_hertz_interval * 8 )) < 65535) {
        // Set CS12, CS11 and CS10 bits for 8 prescaler
        tccr |= (0 << CS12) | (1 << CS11) | (0 << CS10);
    } else if ((ocr = AVR_CLOCK_FREQ / ( tick_hertz_interval * 64 )) < 65535) {
        // Set CS12, CS11 and CS10 bits for 64 prescaler
        tccr |= (0 << CS12) | (1 << CS11) | (1 << CS10);
    } else if ((ocr = AVR_CLOCK_FREQ / ( tick_hertz_interval * 256 )) < 65535) {
        // Set CS12, CS11 and CS10 bits for 256 prescaler
        tccr |= (1 << CS12) | (0 << CS11) | (0 << CS10);
    } else if ((ocr = AVR_CLOCK_FREQ / ( tick_hertz_interval * 1024 )) < 65535) {
        // Set CS12, CS11 and CS10 bits for 1024 prescaler
        tccr |= (1 << CS12) | (0 << CS11) | (1 << CS10);
    } else {
        // tempo not achiavable
        return;
    }

    ATOMIC(
        TCCR1B = 0;
        OCR1A = ocr-1;
        TCCR1B |= (1 << WGM12);
        TCCR1B |= tccr;
    )
}