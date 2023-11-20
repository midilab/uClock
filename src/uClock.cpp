/*!
 *  @file       uClock.cpp
 *  Project     BPM clock generator for Arduino
 *  @brief      A Library to implement BPM clock tick calls using hardware interruption. Supported and tested on AVR boards(ATmega168/328, ATmega16u4/32u4 and ATmega2560) and ARM boards(Teensy, Seedstudio XIAO M0 and ESP32)
 *  @version    1.5.1
 *  @author     Romulo Silva
 *  @date       10/06/2017
 *  @license    MIT - (c) 2022 - Romulo Silva - contact@midilab.co
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE. 
 */
#include "uClock.h"

//
// General Arduino AVRs port
//
#if defined(ARDUINO_ARCH_AVR)
    #include "platforms/avr.h"
#endif
//
// Teensyduino ARMs port
//
#if defined(TEENSYDUINO)
    #include "platforms/teensy.h"
#endif
//
// Seedstudio XIAO M0 port
//
#if defined(SEEED_XIAO_M0)
    #include "platforms/samd.h"
#endif
//
// ESP32 family
//
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    #include "platforms/esp32.h"
#endif
//
// STM32XX family
//
#if defined(ARDUINO_ARCH_STM32)
    #include "platforms/stm32.h"
#endif

//
// Platform specific timer setup/control
//
// initTimer(uint32_t us_interval) and setTimer(uint32_t us_interval)
// are called from architecture specific module included at the
// header of this file
void uclockInitTimer()
{
    // begin at 120bpm (20833us)
    initTimer(20833);
}

void setTimerTempo(float bpm) 
{
    // convert bpm float into 96 ppqn resolution microseconds interval
    uint32_t us_interval = (60000000 / 24 / bpm);
    setTimer(us_interval);
}

namespace umodular { namespace clock {

static inline uint32_t phase_mult(uint32_t val) 
{
    return (val * PHASE_FACTOR) >> 8;
}

static inline uint32_t clock_diff(uint32_t old_clock, uint32_t new_clock) 
{
    if (new_clock >= old_clock) {
        return new_clock - old_clock;
    } else {
        return new_clock + (4294967295 - old_clock);
    }
}

uClockClass::uClockClass()
{
    tempo = 120;
    start_timer = 0;
    last_interval = 0;
    sync_interval = 0;
    state = PAUSED;
    mode = INTERNAL_CLOCK;
    resetCounters();

    onClock96PPQNCallback = NULL;
    onClock32PPQNCallback = NULL;
    onClock16PPQNCallback = NULL;
    onClockStartCallback = NULL;
    onClockStopCallback = NULL;
}

void uClockClass::init() 
{
    uclockInitTimer();
    // first interval calculus
    setTempo(tempo);
}

void uClockClass::start() 
{
    resetCounters();
    start_timer = millis();
    
    if (onClockStartCallback) {
        onClockStartCallback();
    }	
    
    if (mode == INTERNAL_CLOCK) {
        state = STARTED;
    } else {
        state = STARTING;
    }	
}

void uClockClass::stop()
{
    state = PAUSED;
    start_timer = 0;
    resetCounters();
    if (onClockStopCallback) {
        onClockStopCallback();
    }
}

void uClockClass::pause() 
{
    if (mode == INTERNAL_CLOCK) {
        if (state == PAUSED) {
            start();
        } else {
            stop();
        }
    }
}

void uClockClass::setTempo(float bpm) 
{
    if (mode == EXTERNAL_CLOCK) {
        return;
    }
    
    if (bpm < MIN_BPM || bpm > MAX_BPM) {
        return;
    }

    ATOMIC(
        tempo = bpm
    )

    setTimerTempo(bpm);
    
}

float inline uClockClass::freqToBpm(uint32_t freq)
{
    float usecs = 1/((float)freq/1000000.0);
    return (float)((float)(usecs/24.0) * 60.0);
}

float uClockClass::getTempo() 
{
    if (mode == EXTERNAL_CLOCK) {
        uint32_t acc = 0;
        // wait the buffer get full
        if (ext_interval_buffer[EXT_INTERVAL_BUFFER_SIZE-1] == 0) {
            return tempo;
        }
        for (uint8_t i=0; i < EXT_INTERVAL_BUFFER_SIZE; i++) {
            acc += ext_interval_buffer[i];
        }
        if (acc != 0) {
            return freqToBpm(acc / EXT_INTERVAL_BUFFER_SIZE);
        }
    }
    return tempo;
}

void uClockClass::setMode(uint8_t tempo_mode) 
{
    mode = tempo_mode;
}

uint8_t uClockClass::getMode() 
{
    return mode;
}

void uClockClass::clockMe() 
{
    if (mode == EXTERNAL_CLOCK) {
        ATOMIC(
            handleExternalClock()
        )
    }
}

void uClockClass::resetCounters() 
{
    external_clock = 0;
    internal_tick = 0;
    external_tick = 0;
    div32th_counter = 0;
    div16th_counter = 0;
    mod6_counter = 0;	
    indiv32th_counter = 0;
    indiv16th_counter = 0;
    inmod6_counter = 0;
    ext_interval_idx = 0;
    for (uint8_t i=0; i < EXT_INTERVAL_BUFFER_SIZE; i++) {
        ext_interval_buffer[i] = 0;
    }
}

// TODO: Tap stuff
void uClockClass::tap() 
{
    // tap me
}

void uClockClass::setShuffle(bool active)
{
    ATOMIC(shuffle.active = active)
}

bool uClockClass::isShuffled()
{
    return shuffle.active;
}

void uClockClass::setShuffleSize(uint8_t size)
{
    if (size > MAX_SHUFFLE_TEMPLATE_SIZE)
        size = MAX_SHUFFLE_TEMPLATE_SIZE;
    ATOMIC(shuffle.size = size)
}

void uClockClass::setShuffleData(uint8_t step, int8_t tick)
{
    if (step >= MAX_SHUFFLE_TEMPLATE_SIZE)
        return;
    ATOMIC(shuffle.step[step] = tick)
}

void uClockClass::setShuffleTemplate(int8_t * shuff)
{
    uint8_t size = sizeof(shuff) / sizeof(shuff[0]);
    if (size > MAX_SHUFFLE_TEMPLATE_SIZE)
        size = MAX_SHUFFLE_TEMPLATE_SIZE;
    ATOMIC(shuffle.size = size)
    for (uint8_t i=0; i < size; i++) {
        setShuffleData(i, shuff[i]);
    }
}

int8_t uClockClass::getShuffleLength()
{
    return shuffle_length_ctrl;
}

int8_t inline uClockClass::processShuffle()
{
    int8_t mod6_shuffle_counter;
    if (!shuffle.active) {
        mod6_shuffle_counter = mod6_counter;
    } else {
        // apply shuffle template to step
        int8_t shff = shuffle.step[div16th_counter%shuffle.size];
        // keep track of next note shuffle for current note lenght control
        shuffle_length_ctrl = shuffle.step[(div16th_counter+1)%shuffle.size];
        // prepare the next mod6 quantize to be called
        if (shff == 0) {
            mod6_shuffle_counter = mod6_counter;
        } else if (shff > 0) {
            if (shuffle_shoot_ctrl == false && mod6_counter > shff || (shff == 5 && mod6_counter == 0)) 
                shuffle_shoot_ctrl = true;
            mod6_shuffle_counter = shuffle_shoot_ctrl ? mod6_counter - shff : 1;
            shuffle_length_ctrl -= shff;
            if (shuffle_length_ctrl == 0)
                shuffle_length_ctrl = 1;
        } else if (shff < 0) {
            if (shuffle_shoot_ctrl == false && mod6_counter == 0) 
                shuffle_shoot_ctrl = true;
            mod6_shuffle_counter = shff - mod6_counter == -6 ? shuffle_shoot_ctrl ? 0 : 1 : 1;
            shuffle_length_ctrl += shff;
            if (shuffle_length_ctrl == 0)
                shuffle_length_ctrl = -1;
        }
    }
    return mod6_shuffle_counter;
}

void uClockClass::handleExternalClock() 
{

    switch (state) {
        case PAUSED:
            break;

        case STARTING:
            state = STARTED;
            external_clock = micros();
            break;

        case STARTED:

            uint32_t u_timer = micros();
            last_interval = clock_diff(external_clock, u_timer);
            external_clock = u_timer;

            if (inmod6_counter == 0) {
                indiv16th_counter++;
                indiv32th_counter++;
            }

            if (inmod6_counter == 3) {
                indiv32th_counter++;
            }

            // slave tick me!
            external_tick++;
            inmod6_counter++;

            if (inmod6_counter == 6) {
                inmod6_counter = 0;
            }

            // accumulate interval incomming ticks data for getTempo() smooth reads on slave mode
            if(++ext_interval_idx >= EXT_INTERVAL_BUFFER_SIZE) {
                ext_interval_idx = 0;
            }
            ext_interval_buffer[ext_interval_idx] = last_interval;

            if (external_tick == 1) {
                interval = last_interval;
            } else {
                interval = (((uint32_t)interval * (uint32_t)PLL_X) + (uint32_t)(256 - PLL_X) * (uint32_t)last_interval) >> 8;
            }
            break;
    }
}

void uClockClass::handleTimerInt()  
{
    if (mode == EXTERNAL_CLOCK) {
        // sync tick position with external tick clock
        if ((internal_tick < external_tick) || (internal_tick > (external_tick + 1))) {
            internal_tick = external_tick;
            div32th_counter = indiv32th_counter;
            div16th_counter = indiv16th_counter;
            mod6_counter = inmod6_counter;
        }

        uint32_t counter = interval;
        uint32_t u_timer = micros();
        sync_interval = clock_diff(external_clock, u_timer);

        if (internal_tick <= external_tick) {
            counter -= phase_mult(sync_interval);
        } else {
            if (counter > sync_interval) {
                counter += phase_mult(counter - sync_interval);
            }
        }

        // update internal clock timer frequency
        float bpm = freqToBpm(counter);
        if (bpm != tempo) {
            if (bpm >= MIN_BPM && bpm <= MAX_BPM) {
                tempo = bpm;
                setTimerTempo(bpm);
            }
        }
    }

    if (onClock96PPQNCallback) {
        onClock96PPQNCallback(internal_tick);
    }

    // 16PPQN call and shuffle processing if enabled
    if (processShuffle() == 0) {
        if (onClock16PPQNCallback) {
            onClock16PPQNCallback(div16th_counter);
        }
        div16th_counter++;
        shuffle_shoot_ctrl = false;
    }

    // 32PPQN call. does anyone uses it?
    if (mod6_counter == 3 || mod6_counter == 6) {
        if (onClock32PPQNCallback) {
            onClock32PPQNCallback(div32th_counter);
        }
        div32th_counter++;
    }

    // tick me!
    internal_tick++;
    mod6_counter++;

    if (mod6_counter == 6) {
        mod6_counter = 0;
    }
}

// elapsed time support
uint8_t uClockClass::getNumberOfSeconds(uint32_t time)
{
    if ( time == 0 ) {
        return time;
    }
    return ((_timer - time) / 1000) % SECS_PER_MIN;
}

uint8_t uClockClass::getNumberOfMinutes(uint32_t time)
{
    if ( time == 0 ) {
        return time;
    }	
    return (((_timer - time) / 1000) / SECS_PER_MIN) % SECS_PER_MIN;
}

uint8_t uClockClass::getNumberOfHours(uint32_t time)
{
    if ( time == 0 ) {
        return time;
    }	
    return (((_timer - time) / 1000) % SECS_PER_DAY) / SECS_PER_HOUR;
}

uint8_t uClockClass::getNumberOfDays(uint32_t time)
{
    if ( time == 0 ) {
        return time;
    }	
    return ((_timer - time) / 1000) / SECS_PER_DAY;
}

uint32_t uClockClass::getNowTimer()
{
    return _timer;
}
    
uint32_t uClockClass::getPlayTime()
{
    return start_timer;
}
    
} } // end namespace umodular::clock

umodular::clock::uClockClass uClock;

volatile uint32_t _timer = 0;

//
// TIMER INTERRUPT HANDLER 
// 
//
#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER1_COMPA_vect)
#else
void uClockHandler() 
#endif
{
    // global timer counter
    _timer = millis();
    
    if (uClock.state == uClock.STARTED) {
        uClock.handleTimerInt();
    }
}
