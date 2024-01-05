/*!
 *  @file       uClock.cpp
 *  Project     BPM clock generator for Arduino
 *  @brief      A Library to implement BPM clock tick calls using hardware interruption. Supported and tested on AVR boards(ATmega168/328, ATmega16u4/32u4 and ATmega2560) and ARM boards(Teensy, Seedstudio XIAO M0 and ESP32)
 *  @version    2.0.0
 *  @author     Romulo Silva
 *  @date       10/06/2017
 *  @license    MIT - (c) 2024 - Romulo Silva - contact@midilab.co
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
    // begin at 120bpm 
    initTimer(uClock.bpmToMicroSeconds(120.00));
}

void setTimerTempo(float bpm) 
{
    setTimer(uClock.bpmToMicroSeconds(bpm));
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

    onPPQNCallback = nullptr;
    onSync24Callback = nullptr;
    onSync48Callback = nullptr;
    onStepCallback = nullptr;
    onClockStartCallback = nullptr;
    onClockStopCallback = nullptr;
    // first ppqn references calculus
    setPPQN(PPQN_96);
}

void uClockClass::init() 
{
    uclockInitTimer();
    // first interval calculus
    setTempo(tempo);
}

uint32_t uClockClass::bpmToMicroSeconds(float bpm) 
{
    return (60000000 / ppqn / bpm);
}

void uClockClass::setPPQN(PPQNResolution resolution)
{
    // stop clock to make it safe changing those references
    // so we avoid volatile then and ATOMIC everyone
    stop();
    ppqn = resolution;
    // calculate the mod24, mod48 and mod_step tick reference trigger
    mod24_ref = ppqn / 24;
    mod48_ref = ppqn / 48;
    mod_step_ref = ppqn / 4;
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
    return (float)((float)(usecs/(float)ppqn) * 60.0);
}

float uClockClass::getTempo() 
{
    if (mode == EXTERNAL_CLOCK) {
        uint32_t acc = 0;
        // wait the buffer to get full
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

void uClockClass::setMode(SyncMode tempo_mode) 
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
    tick = 0;
    int_clock_tick = 0;
    sync48_tick = 0;
    ext_clock_tick = 0;
    ext_clock_us = 0;
    ext_interval_idx = 0;
    mod24_counter = 0;
    mod48_counter = 0;	
    mod_step_counter = 0;
    step_counter = 0;
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
/* 
int8_t inline uClockClass::processShuffle()
{
    // mod6_counter will become mod_step_counter
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
 */
// it is expected to be called in 24PPQN 
void uClockClass::handleExternalClock() 
{
    switch (state) {
        case PAUSED:
            break;

        case STARTING:
            state = STARTED;
            ext_clock_us = micros();
            break;

        case STARTED:
            uint32_t now_clock_us = micros();
            last_interval = clock_diff(ext_clock_us, now_clock_us);
            ext_clock_us = now_clock_us;

            // external clock tick me!
            ext_clock_tick++;

            // accumulate interval incomming ticks data for getTempo() smooth reads on slave mode
            if(++ext_interval_idx >= EXT_INTERVAL_BUFFER_SIZE) {
                ext_interval_idx = 0;
            }
            ext_interval_buffer[ext_interval_idx] = last_interval;

            if (ext_clock_tick == 1) {
                ext_interval = last_interval;
            } else {
                ext_interval = (((uint32_t)ext_interval * (uint32_t)PLL_X) + (uint32_t)(256 - PLL_X) * (uint32_t)last_interval) >> 8;
            }
            break;
    }
}

void uClockClass::handleTimerInt()  
{
    // External sync is handled here... test if clock check on each tick instead when 
    // mod24_counter kicks in will help or worst slave timing sync quality
    if (mod24_counter == mod24_ref) {
        if (mode == EXTERNAL_CLOCK) {
            // sync tick position with external tick clock
            if ((int_clock_tick < ext_clock_tick) || (int_clock_tick > (ext_clock_tick + 1))) {
                int_clock_tick = ext_clock_tick;
            }

            uint32_t counter = ext_interval;
            uint32_t now_clock_us = micros();
            sync_interval = clock_diff(ext_clock_us, now_clock_us);

            if (int_clock_tick <= ext_clock_tick) {
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
        // callback to inform about sync24 event
        if (onSync24Callback) {
            onSync24Callback(int_clock_tick);
        }
        // reset counter
        mod24_counter = 0;
        // internal clock tick me! sync24 tick too
        ++int_clock_tick;
    }

    // sync signals first please...
    if (onSync48Callback) {
        if (mod48_counter == mod48_ref) {
            onSync48Callback(sync48_tick);
            // reset counter
            mod48_counter = 0;
            // sync48 tick me!
            ++sync48_tick;
        }
    }

    // PPQNCallback time!
    if (onPPQNCallback) {
        onPPQNCallback(tick);
    }
    
    if (onStepCallback) {
        // we can add a time signature here for call setup based on mod_step_ref
        // basic will be 16ths, but let the option to handle unusual sequences
        if (mod_step_counter == mod_step_ref) {
            onStepCallback(step_counter);
            // reset counter
            mod_step_counter = 0;
            // going forward to the next step call
            ++step_counter;
        }
    }

    /* // TODO: port it from 24PPQN to ppqn set
    if (processShuffle() == 0) {        
        //if (mod_step_counter == signature) {
        if (onStepCallback) {
            onStepCallback(mod_step_counter);
        }
        shuffle_shoot_ctrl = false;
    } */

    // tick me!
    ++tick;
    // increment mod counters
    ++mod24_counter;
    ++mod48_counter;
    ++mod_step_counter;
}

// elapsed time support
uint8_t uClockClass::getNumberOfSeconds(uint32_t time)
{
    if ( time == 0 ) {
        return time;
    }
    return ((_millis - time) / 1000) % SECS_PER_MIN;
}

uint8_t uClockClass::getNumberOfMinutes(uint32_t time)
{
    if ( time == 0 ) {
        return time;
    }	
    return (((_millis - time) / 1000) / SECS_PER_MIN) % SECS_PER_MIN;
}

uint8_t uClockClass::getNumberOfHours(uint32_t time)
{
    if ( time == 0 ) {
        return time;
    }	
    return (((_millis - time) / 1000) % SECS_PER_DAY) / SECS_PER_HOUR;
}

uint8_t uClockClass::getNumberOfDays(uint32_t time)
{
    if ( time == 0 ) {
        return time;
    }	
    return ((_millis - time) / 1000) / SECS_PER_DAY;
}

uint32_t uClockClass::getNowTimer()
{
    return _millis;
}
    
uint32_t uClockClass::getPlayTime()
{
    return start_timer;
}
    
} } // end namespace umodular::clock

umodular::clock::uClockClass uClock;

volatile uint32_t _millis = 0;

//
// TIMER HANDLER 
// 
#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER1_COMPA_vect)
#else
void uClockHandler() 
#endif
{
    // global timer counter
    _millis = millis();
    
    if (uClock.state == uClock.STARTED) {
        uClock.handleTimerInt();
    }
}
