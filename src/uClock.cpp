/*!
 *  @file       uClock.cpp
 *  Project     BPM clock generator for Arduino
 *  @brief      A Library to implement BPM clock tick calls using hardware interruption. Supported and tested on AVR boards(ATmega168/328, ATmega16u4/32u4 and ATmega2560) and ARM boards(RPI2040, Teensy, Seedstudio XIAO M0 and ESP32)
 *  @version    2.3.0
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
// Compile time selection of Platform implementation of timer setup/control/handler
//
#if !defined(USE_UCLOCK_SOFTWARE_TIMER)
    //
    // General Arduino AVRs port
    //
    #if defined(ARDUINO_ARCH_AVR)
        #include "platforms/avr.h"
        #define UCLOCK_PLATFORM_FOUND
    #endif
    //
    // Teensyduino ARMs port
    //
    #if defined(TEENSYDUINO)
        #include "platforms/teensy.h"
        #define UCLOCK_PLATFORM_FOUND
    #endif
    //
    // Seedstudio XIAO M0 port
    //
    #if defined(SEEED_XIAO_M0)
        #include "platforms/samd.h"
        #define UCLOCK_PLATFORM_FOUND
    #endif
    //
    // ESP32 family
    //
    #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        #include "platforms/esp32.h"
        #define UCLOCK_PLATFORM_FOUND
    #endif
    //
    // STM32XX family
    //
    #if defined(ARDUINO_ARCH_STM32)
        #include "platforms/stm32.h"
        #define UCLOCK_PLATFORM_FOUND
    #endif
    //
    // RP2040 (Raspberry Pico) family
    //
    #if defined(ARDUINO_ARCH_RP2040)
        #include "platforms/rp2040.h"
        #define UCLOCK_PLATFORM_FOUND
    #endif
#endif

//
// Software Timer for generic, board-agnostic, not-accurate, no-interrupt, software-only port
// No hardware timer support? fallback to USE_UCLOCK_SOFTWARE_TIMER
//
#if !defined(UCLOCK_PLATFORM_FOUND)
    #pragma message ("NOTE: uClock is using the 'software timer' approach instead of specific board interrupted support, because board is not supported or because of USE_UCLOCK_SOFTWARE_TIMER build flag. Remember to call uClock.run() inside your loop().")
    #include "platforms/software.h"
#endif

//
// Platform specific timer handler/setup/control wrappers
//
// global timer counter
volatile uint32_t _millis = 0;

// called each tick genarated from platform specific timer
void uClockHandler()
{
    _millis = millis();

    if (uClock.clock_state == uClock.STARTED || uClock.clock_state == uClock.SYNCING)
        uClock.handleInternalClock();
}

// initTimer(uint32_t us_interval) and setTimer(uint32_t us_interval)
// are defined at platform specific code and are platform dependent
void uClockInitTimer()
{
    // initialize at 120bpm as default
    initTimer(uClock.bpmToMicroSeconds(120.00));
}

void uClockSetTimerTempo(float bpm)
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
    resetCounters();

    // initialize reference data
    calculateReferencedata();
}

uClockClass::~uClockClass()
{
    delete[] ext_interval_buffer;
}

void uClockClass::init()
{
    if (ext_interval_buffer == nullptr)
        setExtIntervalBuffer(1);

    uClockInitTimer();
    // first interval calculus
    setTempo(tempo);
}

void uClockClass::handleInternalClock()
{
    // track main input clock counter
    if (mod_clock_counter == mod_clock_ref)
        mod_clock_counter = 0;

    // process sync signals first please...
    if (mod_clock_counter == 0) {

        if (clock_mode == EXTERNAL_CLOCK && clock_state == STARTED) {
            // sync tick position with external tick clock
            if ((int_clock_tick < ext_clock_tick) || (int_clock_tick > (ext_clock_tick + 1))) {
                int_clock_tick = ext_clock_tick;
                tick = int_clock_tick * mod_clock_ref;
                mod_clock_counter = tick % mod_clock_ref;
                mod_step_counter = tick % mod_step_ref;
            }

            hlp_counter = ext_interval;
            hlp_sync_interval = clock_diff(ext_clock_us, micros());

            if (int_clock_tick <= ext_clock_tick) {
                hlp_counter -= phase_mult(hlp_sync_interval);
            } else {
                if (hlp_counter > hlp_sync_interval) {
                    hlp_counter += phase_mult(hlp_counter - hlp_sync_interval);
                }
            }

            // update internal clock timer frequency
            hlp_external_bpm = constrainBpm(freqToBpm(hlp_counter));
            if (hlp_external_bpm != tempo) {
                tempo = hlp_external_bpm;
                uClockSetTimerTempo(tempo);
            }
        }

        // internal clock tick me!
        ++int_clock_tick;
    }
    ++mod_clock_counter;

    // ALL OUTPUT SYNC CALLBACKS
    // Sync1 callback
    if (onSync1Callback) {
        if (mod_sync1_counter == mod_sync1_ref)
            mod_sync1_counter = 0;
        if (mod_sync1_counter == 0) {
            onSync1Callback(sync1_tick);
            ++sync1_tick;
        }
        ++mod_sync1_counter;
    }

    // Sync2 callback
    if (onSync2Callback) {
        if (mod_sync2_counter == mod_sync2_ref)
            mod_sync2_counter = 0;
        if (mod_sync2_counter == 0) {
            onSync2Callback(sync2_tick);
            ++sync2_tick;
        }
        ++mod_sync2_counter;
    }

    // Sync4 callback
    if (onSync4Callback) {
        if (mod_sync4_counter == mod_sync4_ref)
            mod_sync4_counter = 0;
        if (mod_sync4_counter == 0) {
            onSync4Callback(sync4_tick);
            ++sync4_tick;
        }
        ++mod_sync4_counter;
    }

    // Sync8 callback
    if (onSync8Callback) {
        if (mod_sync8_counter == mod_sync8_ref)
            mod_sync8_counter = 0;
        if (mod_sync8_counter == 0) {
            onSync8Callback(sync8_tick);
            ++sync8_tick;
        }
        ++mod_sync8_counter;
    }

    // Sync12 callback
    if (onSync12Callback) {
        if (mod_sync12_counter == mod_sync12_ref)
            mod_sync12_counter = 0;
        if (mod_sync12_counter == 0) {
            onSync12Callback(sync12_tick);
            ++sync12_tick;
        }
        ++mod_sync12_counter;
    }

    // Sync24 callback
    if (onSync24Callback) {
        if (mod_sync24_counter == mod_sync24_ref)
            mod_sync24_counter = 0;
        if (mod_sync24_counter == 0) {
            onSync24Callback(sync24_tick);
            ++sync24_tick;
        }
        ++mod_sync24_counter;
    }

    // Sync48 callback
    if (onSync48Callback) {
        if (mod_sync48_counter == mod_sync48_ref)
            mod_sync48_counter = 0;
        if (mod_sync48_counter == 0) {
            onSync48Callback(sync48_tick);
            ++sync48_tick;
        }
        ++mod_sync48_counter;
    }

    // main PPQNCallback
    if (onOutputPPQNCallback) {
        onOutputPPQNCallback(tick);
        ++tick;
    }

    // step callback to support 16th old school style sequencers
    // with builtin shuffle for this callback only
    if (onStepCallback) {
        if (mod_step_counter == mod_step_ref)
            mod_step_counter = 0;
        // processShufle make use of mod_step_counter == 0 logic too
        if (processShuffle()) {
            onStepCallback(step_counter);
            // going forward to the next step call
            ++step_counter;
        }
        ++mod_step_counter;
    }
}

void uClockClass::handleExternalClock()
{
    switch (clock_state) {
        case SYNCING:
            // set clock_mode as started, and goes on to calculate the first ext_interval
            clock_state = STARTED;
            // no break here, just go on to calculate our first ext_interval

        case STARTED:
            hlp_now_clock_us = micros();
            hlp_last_interval = clock_diff(ext_clock_us, hlp_now_clock_us);
            ext_clock_us = hlp_now_clock_us;

            // accumulate interval incomming ticks data for getTempo() smooth reads on slave clock_mode
            if(++ext_interval_idx >= ext_interval_buffer_size)
                ext_interval_idx = 0;
            ext_interval_buffer[ext_interval_idx] = hlp_last_interval;

            // external clock tick me!
            ext_clock_tick++;

            // set sync interval
            if (ext_clock_tick == 1) {
                ext_interval = hlp_last_interval;
            } else {
                ext_interval = (((uint32_t)ext_interval * (uint32_t)PLL_X) + (uint32_t)(256 - PLL_X) * (uint32_t)hlp_last_interval) >> 8;
            }
            break;

        case STOPED:
        case PAUSED:
            break;

        case STARTING:
            clock_state = SYNCING;
            ext_clock_us = micros();
            break;
    }
}

void uClockClass::clockMe()
{
    ATOMIC(handleExternalClock())
}

void uClockClass::start()
{
    resetCounters();
    start_timer = millis();

    if (onClockStartCallback)
        onClockStartCallback();

    if (clock_mode == INTERNAL_CLOCK) {
        ATOMIC(clock_state = STARTED)
    } else {
        ATOMIC(clock_state = STARTING)
    }
}

void uClockClass::stop()
{
    ATOMIC(clock_state = STOPED)
    resetCounters();
    start_timer = 0;
    if (onClockStopCallback)
        onClockStopCallback();
}

void uClockClass::pause()
{
    if (clock_state == STARTED) {
        ATOMIC(clock_state = PAUSED)
        if (onClockPauseCallback)
            onClockPauseCallback();
    } else if (clock_state == PAUSED) {
        if (clock_mode == INTERNAL_CLOCK) {
            ATOMIC(clock_state = STARTED)
        } else if (clock_mode == EXTERNAL_CLOCK) {
            ATOMIC(clock_state = STARTING)
        }
        if (onClockContinueCallback)
            onClockContinueCallback();
    }
}

void uClockClass::setClockMode(ClockMode tempo_mode)
{
    ATOMIC(
        clock_mode = tempo_mode;
        // trying to set external clock while playing? force sync ext_interval
        if (clock_mode == EXTERNAL_CLOCK && clock_state == STARTED)
            clock_state = STARTING;
    )
}

uClockClass::ClockMode uClockClass::getClockMode()
{
    return clock_mode;
}

uint32_t uClockClass::bpmToMicroSeconds(float bpm)
{
    return (60000000.0f / (float)output_ppqn / bpm);
}

void uClockClass::calculateReferencedata()
{
    mod_clock_ref = output_ppqn / input_ppqn;
    mod_sync1_ref = output_ppqn / PPQN_1;
    mod_sync2_ref = output_ppqn / PPQN_2;
    mod_sync4_ref = output_ppqn / PPQN_4;
    mod_sync8_ref = output_ppqn / PPQN_8;
    mod_sync12_ref = output_ppqn / PPQN_12;
    mod_sync24_ref = output_ppqn / PPQN_24;
    mod_sync48_ref = output_ppqn / PPQN_48;
    mod_step_ref = output_ppqn / 4;
}

void uClockClass::setOutputPPQN(PPQNResolution resolution)
{
    // dont allow PPQN lower than PPQN_4 for output clock (to avoid problems with mod_step_ref)
    if (resolution < PPQN_4)
        return;

    ATOMIC(
        output_ppqn = resolution;
        calculateReferencedata();
    )
}

void uClockClass::setInputPPQN(PPQNResolution resolution)
{
    ATOMIC(
        input_ppqn = resolution;
        calculateReferencedata();
    )
}

void uClockClass::setTempo(float bpm)
{
    if (clock_mode == EXTERNAL_CLOCK) {
        return;
    }

    if (bpm < MIN_BPM || bpm > MAX_BPM) {
        return;
    }

    ATOMIC(tempo = bpm)

    uClockSetTimerTempo(bpm);
}

float uClockClass::getTempo()
{
    if (clock_mode == EXTERNAL_CLOCK) {
        uint32_t acc = 0;
        // wait the buffer to get full
        if (ext_interval_buffer[ext_interval_buffer_size-1] == 0) {
            return tempo;
        }
        for (uint8_t i=0; i < ext_interval_buffer_size; i++) {
            acc += ext_interval_buffer[i];
        }
        if (acc != 0) {
            return constrainBpm(freqToBpm(acc / ext_interval_buffer_size));
        }
    }
    return tempo;
}

// for software timer implementation(fallback for no board support)
void uClockClass::run()
{
#if !defined(UCLOCK_PLATFORM_FOUND)
    // call software timer implementation of software
    softwareTimerHandler(micros());
#endif
}

float inline uClockClass::freqToBpm(uint32_t freq)
{
    float usecs = 1/((float)freq/1000000.0);
    return (float)((float)(usecs/(float)input_ppqn) * 60.0);
}

float inline uClockClass::constrainBpm(float bpm)
{
    return (bpm < MIN_BPM) ? MIN_BPM : ( bpm > MAX_BPM ? MAX_BPM : bpm );
}

void uClockClass::setExtIntervalBuffer(uint8_t buffer_size)
{
    if (ext_interval_buffer != nullptr)
        return;

    // alloc once and forever policy
    ext_interval_buffer_size = buffer_size;
    ext_interval_buffer = new uint32_t[ext_interval_buffer_size];
}

void uClockClass::resetCounters()
{
    tick = 0;
    int_clock_tick = 0;
    mod_clock_counter = 0;
    mod_step_counter = 0;
    step_counter = 0;
    ext_clock_tick = 0;
    ext_clock_us = 0;
    ext_interval_idx = 0;
    // sync output counters
    mod_sync1_counter = 0;
    sync1_tick = 0;
    mod_sync2_counter = 0;
    sync2_tick = 0;
    mod_sync4_counter = 0;
    sync4_tick = 0;
    mod_sync8_counter = 0;
    sync8_tick = 0;
    mod_sync12_counter = 0;
    sync12_tick = 0;
    mod_sync24_counter = 0;
    sync24_tick = 0;
    mod_sync48_counter = 0;
    sync48_tick = 0;

    for (uint8_t i=0; i < ext_interval_buffer_size; i++) {
        ext_interval_buffer[i] = 0;
    }
}

void uClockClass::tap()
{
    // we can make use of mod_sync1_ref for tap
    //uint8_t mod_tap_ref = output_ppqn / PPQN_1;
    // we only set tap if ClockMode is INTERNAL_CLOCK
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

void uClockClass::setShuffleTemplate(int8_t * shuff, uint8_t size)
{
    //uint8_t size = sizeof(shuff) / sizeof(shuff[0]);
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

bool inline uClockClass::processShuffle()
{
    if (!shuffle.active) {
        return mod_step_counter == 0;
    }

    int8_t mod_shuffle = 0;

    // check shuffle template of current
    int8_t shff = shuffle.step[step_counter%shuffle.size];

    if (shuffle_shoot_ctrl == false && mod_step_counter == 0)
        shuffle_shoot_ctrl = true;

    //if (mod_step_counter == mod_step_ref-1)

    if (shff >= 0) {
        mod_shuffle = mod_step_counter - shff;
        // any late shuffle? we should skip next mod_step_counter == 0
        if (last_shff < 0 && mod_step_counter != 1)
            return false;
    } else if (shff < 0) {
        mod_shuffle = mod_step_counter - (mod_step_ref + shff);
        //if (last_shff < 0 && mod_step_counter != 1)
        //    return false;
        shuffle_shoot_ctrl = true;
    }

    last_shff = shff;

    // shuffle_shoot_ctrl helps keep track if we have shoot or not a note for the step space of output_ppqn/4 pulses
    if (mod_shuffle == 0 && shuffle_shoot_ctrl == true) {
        // keep track of next note shuffle for current note lenght control
        shuffle_length_ctrl = shuffle.step[(step_counter+1)%shuffle.size];
        if (shff > 0)
            shuffle_length_ctrl -= shff;
        if (shff < 0)
            shuffle_length_ctrl += shff;
        shuffle_shoot_ctrl = false;
        return true;
    }

    return false;
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
