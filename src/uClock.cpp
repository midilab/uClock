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

    if (uClock.clock_state == uClock.STARTED || uClock.clock_state == uClock.STARTING)
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
    if (ext_interval_buffer)
        delete[] ext_interval_buffer;

    if (tracks)
        delete[] tracks;
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
    // ALL OUTPUT SYNC CALLBACKS
    // Sync1 callback
    if (onSync1Callback) {
        if (tick % mod_sync1_ref == 0) {
            onSync1Callback(sync1_tick);
            ++sync1_tick;
        }
    }

    // Sync2 callback
    if (onSync2Callback) {
        if (tick % mod_sync2_ref == 0) {
            onSync2Callback(sync2_tick);
            ++sync2_tick;
        }
    }

    // Sync4 callback
    if (onSync4Callback) {
        if (tick % mod_sync4_ref == 0) {
            onSync4Callback(sync4_tick);
            ++sync4_tick;
        }
    }

    // Sync8 callback
    if (onSync8Callback) {
        if (tick % mod_sync8_ref == 0) {
            onSync8Callback(sync8_tick);
            ++sync8_tick;
        }
    }

    // Sync12 callback
    if (onSync12Callback) {
        if (tick % mod_sync12_ref == 0) {
            onSync12Callback(sync12_tick);
            ++sync12_tick;
        }
    }

    // Sync24 callback
    if (onSync24Callback) {
        if (tick % mod_sync24_ref == 0) {
            onSync24Callback(sync24_tick);
            ++sync24_tick;
        }
    }

    // Sync48 callback
    if (onSync48Callback) {
        if (tick % mod_sync48_ref == 0) {
            onSync48Callback(sync48_tick);
            ++sync48_tick;
        }
    }

    // StepSeq extension: step callback to support 16th old school style sequencers
    // with builtin shuffle
    if (tracks != nullptr) {
        // process onStepCallback()
        stepSeqTick();
    }

    // main PPQNCallback
    if (onOutputPPQNCallback) {
        onOutputPPQNCallback(tick);
        ++tick;
    }

    // reference internal clock to use with external clock tick sync
    if (tick % mod_clock_ref == 0) {
        // internal clock tick me!
        ++int_clock_tick;
    }
}

void uClockClass::handleExternalClock()
{
    switch (clock_state) {
        case STARTED:
            hlp_now_clock_us = micros();
            ext_interval = clock_diff(ext_clock_us, hlp_now_clock_us);
            ext_clock_us = hlp_now_clock_us;

            // update internal clock timer frequency
            if (clock_mode == EXTERNAL_CLOCK) {
                hlp_external_bpm = constrainBpm(freqToBpm(ext_interval));
                if (hlp_external_bpm != tempo) {
                    tempo = hlp_external_bpm;
                    uClockSetTimerTempo(tempo);
                }
                // TODO: missing clock tick inference algorithim
                // to avoid tick miss sync with external quarter note start
                // bad comm, missing a local read, bad sync signal? miss 1 clock and things goes off quarter start beat.
                // or maybe this could be a optional external/internal clock phase lock?
                //
                // ideal world here we should have ext_clock_tick == int_clock_tick
                // wich means each external tick subsequent internal tick should be
                // fired and fire again only after another external tick income.
                // lock tick with external sync clock
                // sync tick position with external clock signal
                if (clock_diff(int_clock_tick, ext_clock_tick) > 1) {
                    // only update tick at a full quarter start based on output_ppqn to avoid step quarter phase sync problems
                    if (tick % output_ppqn == 0) {
                        int_clock_tick = ext_clock_tick;
                        tick = int_clock_tick * mod_clock_ref;
                    }
                }
            }

            // accumulate interval incomming ticks data for getTempo() smooth reads on slave clock_mode
            ext_interval_buffer[ext_interval_idx] = ext_interval;
            if(++ext_interval_idx >= ext_interval_buffer_size)
                ext_interval_idx = 0;
            break;

        case STARTING:
            clock_state = STARTED;
            ext_clock_us = micros();
            ext_clock_tick = 0;
            int_clock_tick = 0;
            break;

        case STOPED:
        case PAUSED:
            break;
    }

    // external clock tick me!
    ext_clock_tick++;
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

// for software timer implementation(fallback for no timer board support)
void uClockClass::run()
{
#if !defined(UCLOCK_PLATFORM_FOUND)
    // call software timer implementation
    softwareTimerHandler(micros());
#endif
}

void uClockClass::stepSeqTick()
{
    for (uint8_t track=0; track < track_slots_size; track++) {
        bool stepProcess = false;
        if (!tracks[track].shuffle.tmplt.active) {
            if (tick % mod_step_ref == 0)
                stepProcess = true;
        } else if (processShuffle(track)) {
            stepProcess = true;
        }

        if (stepProcess) {
            if (onStepGlobalCallback)
                onStepGlobalCallback(tracks[track].step_counter);
            if (onStepMultiCallback)
                onStepMultiCallback(tracks[track].step_counter, track);

            // going forward to the next step call
            ++tracks[track].step_counter;
        }
    }
}

void uClockClass::setShuffle(bool active, uint8_t track)
{
    if (tracks == nullptr)
        return;

    ATOMIC(tracks[track].shuffle.tmplt.active = active)
}

bool uClockClass::isShuffled(uint8_t track)
{
    if (tracks == nullptr)
        return false;

    return tracks[track].shuffle.tmplt.active;
}

void uClockClass::setShuffleSize(uint8_t size, uint8_t track)
{
    if (tracks == nullptr)
        return;

    if (size > MAX_SHUFFLE_TEMPLATE_SIZE)
        size = MAX_SHUFFLE_TEMPLATE_SIZE;
    ATOMIC(tracks[track].shuffle.tmplt.size = size)
}

void uClockClass::setShuffleData(uint8_t step, int8_t tick, uint8_t track)
{
    if (tracks == nullptr)
        return;

    if (step >= MAX_SHUFFLE_TEMPLATE_SIZE)
        return;
    ATOMIC(tracks[track].shuffle.tmplt.step[step] = tick)
}

void uClockClass::setShuffleTemplate(int8_t * shuff, uint8_t size, uint8_t track)
{
    if (tracks == nullptr)
        return;

    //uint8_t size = sizeof(shuff) / sizeof(shuff[0]);
    if (size > MAX_SHUFFLE_TEMPLATE_SIZE)
        size = MAX_SHUFFLE_TEMPLATE_SIZE;
    ATOMIC(tracks[track].shuffle.tmplt.size = size)
    for (uint8_t i=0; i < size; i++) {
        setShuffleData(i, shuff[i], track);
    }
}

int8_t uClockClass::getShuffleLength(uint8_t track)
{
    if (tracks == nullptr)
        return 0;

    return tracks[track].shuffle.shuffle_length_ctrl;
}

bool inline uClockClass::processShuffle(uint8_t track)
{
    if (tracks == nullptr)
        return false;

    int8_t mod_shuffle = 0;

    // check shuffle template of current
    int8_t shff = tracks[track].shuffle.tmplt.step[tracks[track].step_counter%tracks[track].shuffle.tmplt.size];

    if (tracks[track].shuffle.shuffle_shoot_ctrl == false && tick % mod_step_ref == 0)
        tracks[track].shuffle.shuffle_shoot_ctrl = true;

    if (shff >= 0) {
        mod_shuffle = (tick % mod_step_ref) - shff;
        // any late shuffle? we should skip next tracks[track].mod_step_counter == 0
        if (tracks[track].shuffle.last_shff < 0 && tick % mod_step_ref != 1) {
            if (tracks[track].shuffle.shuffle_shoot_ctrl == true)
                tracks[track].shuffle.shuffle_shoot_ctrl = false;
            return false;
        }
    } else if (shff < 0) {
        mod_shuffle = (tick % mod_step_ref) - (mod_step_ref + shff);
        tracks[track].shuffle.shuffle_shoot_ctrl = true;
    }

    tracks[track].shuffle.last_shff = shff;

    // shuffle_shoot_ctrl helps keep track if we have shoot or not a note for the step space of output_ppqn/4 pulses
    if (mod_shuffle == 0 && tracks[track].shuffle.shuffle_shoot_ctrl == true) {
        // keep track of next note shuffle for current note lenght control
        tracks[track].shuffle.shuffle_length_ctrl = tracks[track].shuffle.tmplt.step[(tracks[track].step_counter+1)%tracks[track].shuffle.tmplt.size];
        if (shff > 0)
            tracks[track].shuffle.shuffle_length_ctrl -= shff;
        if (shff < 0)
            tracks[track].shuffle.shuffle_length_ctrl += shff;
        tracks[track].shuffle.shuffle_shoot_ctrl = false;
        return true;
    }

    return false;
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

    // dont allow output_ppqn lower than input_ppqn
    if (resolution < input_ppqn)
        return;

    ATOMIC(
        output_ppqn = resolution;
        calculateReferencedata();
    )
}

void uClockClass::setInputPPQN(PPQNResolution resolution)
{
    // dont allow input_ppqn greater than output_ppqn
    if (resolution > output_ppqn)
        return;

    ATOMIC(
        input_ppqn = resolution;
        calculateReferencedata();
    )
}

void uClockClass::setTempo(float bpm)
{
    if (clock_mode == EXTERNAL_CLOCK)
        return;

    if (bpm < MIN_BPM || bpm > MAX_BPM)
        return;

    ATOMIC(tempo = bpm)

    uClockSetTimerTempo(bpm);
}

float uClockClass::getTempo()
{
    if (clock_mode == EXTERNAL_CLOCK) {
        uint32_t acc = 0;
        for (uint8_t i=0; i < ext_interval_buffer_size; i++)
            acc += ext_interval_buffer[i];
        return constrainBpm(freqToBpm(acc / ext_interval_buffer_size));
    }
    return tempo;
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

void uClockClass::setExtIntervalBuffer(size_t buffer_size)
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
    ext_clock_tick = 0;
    ext_clock_us = 0;
    ext_interval = 0;
    ext_interval_idx = 0;
    // sync output counters
    sync1_tick = 0;
    sync2_tick = 0;
    sync4_tick = 0;
    sync8_tick = 0;
    sync12_tick = 0;
    sync24_tick = 0;
    sync48_tick = 0;

    for (uint8_t track=0; track < track_slots_size; track++) {
        tracks[track].step_counter = 0;
    }

    //for (uint8_t i=0; i < ext_interval_buffer_size; i++) {
    //    ext_interval_buffer[i] = 0;
    //}
}

void uClockClass::tap()
{
    // we can make use of mod_sync1_ref for tap
    //uint8_t mod_tap_ref = output_ppqn / PPQN_1;
    // we only set tap if ClockMode is INTERNAL_CLOCK
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
