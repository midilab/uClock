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
        return new_clock + (4294967295UL - old_clock);
    }
}

uClockClass::uClockClass()
{
    resetCounters();
}

uClockClass::~uClockClass()
{
    if (sync_callbacks)
        delete[] sync_callbacks;

    if (ext_interval_buffer)
        delete[] ext_interval_buffer;

    if (tracks)
        delete[] tracks;
}

void uClockClass::init()
{
    if (ext_interval_buffer == nullptr)
        setExtIntervalBuffer(1);

    // initialize reference data
    calculateReferencedata();
    // initialize hardware timer
    uClockInitTimer();
    // first interval calculus
    setTempo(tempo);
}

void uClockClass::handleInternalClock()
{
    static uint32_t counter = 0;
    static uint32_t sync_interval = 0;

    // for debug usage while developing any application under uClock
    ++int_overflow_counter;

    if (clock_state <= STARTING) // STOPED=0, PAUSED=1, STARTING=2, SYNCING=3, STARTED=4
        return;

    // tick phase lock and external tempo match for EXTERNAL_CLOCK mode
    if (clock_mode == EXTERNAL_CLOCK) {
        // Tick Phase-lock
        if (labs(int_clock_tick - ext_clock_tick) > 1) {
            // only update tick at a full quarter or phase_lock_quarters * a quarter
            // how many quarters to count until we phase-lock?
            if ((ext_clock_tick * mod_clock_ref) % (output_ppqn*phase_lock_quarters) == 0) {
                tick = ext_clock_tick * mod_clock_ref;
                int_clock_tick = ext_clock_tick;
                // update any counter reference to lock with int_clock_tick
                for (uint8_t track=0; track < track_slots_size; track++) {
                    tracks[track].step_counter = tick/mod_step_ref;
                    tracks[track].mod_step_counter = 0;
                }
                // update counter reference for sync callbacks
                for (uint8_t i = 0; i < sync_callback_size; i++) {
                    if (sync_callbacks[i].callback) {
                        sync_callbacks[i].tick = tick/sync_callbacks[i].sync_ref;
                        sync_callbacks[i].mod_counter = 0;
                    }
                }
            }
        }

        // any external interval avaliable to start sync timer?
        if (ext_interval > 0) {
            counter = ext_interval;
            sync_interval = clock_diff(ext_clock_us, micros());

            // phase-multiplier interval
            if (int_clock_tick <= ext_clock_tick) {
                counter -= (sync_interval * PHASE_FACTOR) >> 8;
            } else {
                if (counter > sync_interval) {
                    counter += ((counter - sync_interval) * PHASE_FACTOR) >> 8;
                }
            }

            external_tempo = constrainBpm(freqToBpm(counter));
            if (external_tempo != tempo) {
                tempo = external_tempo;
                uClockSetTimerTempo(tempo);
            }
        }
    }

    // main input clock counter control
    if (mod_clock_counter == mod_clock_ref)
        mod_clock_counter = 0;
    // process internal clock signal
    // int_clock_tick is the internal clock reference. mainly used for external clock phase lock
    if (mod_clock_counter == 0)
        ++int_clock_tick;
    ++mod_clock_counter;

    // sync callbacks
    for (uint8_t i = 0; i < sync_callback_size; i++) {
        if (sync_callbacks[i].mod_counter == sync_callbacks[i].sync_ref)
            sync_callbacks[i].mod_counter = 0;
        if (sync_callbacks[i].mod_counter == 0) {
            sync_callbacks[i].callback(sync_callbacks[i].tick);
            // tick sync callback
            ++sync_callbacks[i].tick;
        }
        ++sync_callbacks[i].mod_counter;
    }

    // StepSeq extension: step callback to support 16th old school style sequencers
    // with builtin shuffle - process onStepCallback()
    if (tracks)
        stepSeqTick();

    // main PPQNCallback
    if (onOutputPPQNCallback)
        onOutputPPQNCallback(tick);

    // internal ticking
    ++tick;

    // for debug usage while developing any application under uClock
    --int_overflow_counter;
}

void uClockClass::handleExternalClock()
{
    static uint32_t now_clock_us = 0;
    static uint8_t start_sync_counter = 0;

    // for debug usage while developing any application under uClock
    ++ext_overflow_counter;

    // calculate and store ext_interval
    now_clock_us = micros();
    if (ext_clock_us > 0)
        ext_interval = clock_diff(ext_clock_us, now_clock_us);
    ext_clock_us = now_clock_us;

    // external clock tick me!
    ext_clock_tick++;

    switch (clock_state) {
        case STARTING:
            clock_state = SYNCING;
            start_sync_counter = 4;
            break;
        case SYNCING:
            if (--start_sync_counter == 0)
                clock_state = STARTED;
            break;
        default:
            // accumulate interval incomming ticks data for getTempo() smooth reads on slave clock_mode
            if (ext_interval > 0) {
                ext_interval_buffer[ext_interval_idx] = ext_interval;
                if(++ext_interval_idx >= ext_interval_buffer_size)
                    ext_interval_idx = 0;
            }
            break;
    }

    // for debug usage while developing any application under uClock
    --ext_overflow_counter;
}

void uClockClass::clockMe()
{
    ATOMIC(handleExternalClock())
}

void uClockClass::start()
{
    ATOMIC(resetCounters())
    start_timer = millis();

    if (clock_mode == INTERNAL_CLOCK) {
        ATOMIC(clock_state = STARTED)
    } else {
        ATOMIC(clock_state = STARTING)
    }

    if (onClockStartCallback)
        onClockStartCallback();
}

void uClockClass::stop()
{
    ATOMIC(clock_state = STOPED)
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
        // trying to set external clock while playing?
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
        if (tracks[track].mod_step_counter == mod_step_ref)
            tracks[track].mod_step_counter = 0;
        if (!tracks[track].shuffle.tmplt.active) {
            if (tracks[track].mod_step_counter == 0)
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
        ++tracks[track].mod_step_counter;
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

    if (tracks[track].shuffle.shuffle_shoot_ctrl == false && tracks[track].mod_step_counter == 0)
        tracks[track].shuffle.shuffle_shoot_ctrl = true;

    if (shff >= 0) {
        mod_shuffle = tracks[track].mod_step_counter - shff;
        // any late shuffle? we should skip next tracks[track].mod_step_counter == 0
        if (tracks[track].shuffle.last_shff < 0 && tracks[track].mod_step_counter != 1) {
            if (tracks[track].shuffle.shuffle_shoot_ctrl == true)
                tracks[track].shuffle.shuffle_shoot_ctrl = false;
            return false;
        }
    } else if (shff < 0) {
        mod_shuffle = tracks[track].mod_step_counter - (mod_step_ref + shff);
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
    mod_step_ref = output_ppqn / 4;
    // sync callback references update
    for (uint8_t i = 0; i < sync_callback_size; i++)
        sync_callbacks[i].sync_ref = output_ppqn / sync_callbacks[i].resolution;
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
        uint64_t acc = 0;
        uint8_t valid_buffer_size = 0;
        for (uint8_t i=0; i < ext_interval_buffer_size; i++) {
            if (ext_interval_buffer[i] > 0) {
                ATOMIC(acc += ext_interval_buffer[i])
                ++valid_buffer_size;
            }
        }
        if (acc == 0)
            return tempo;
        return constrainBpm(freqToBpm(acc / valid_buffer_size));
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

    for (uint8_t i=0; i < ext_interval_buffer_size; i++)
        ext_interval_buffer[i] = 0;
}

void uClockClass::setPhaseLockQuartersCount(uint8_t count)
{
    ATOMIC(phase_lock_quarters = count)
}

void uClockClass::resetCounters()
{
    tick = 0;
    mod_clock_counter = 0;
    int_clock_tick = 0;
    ext_clock_tick = 0;
    ext_clock_us = 0;
    ext_interval = 0;
    //ext_interval_idx = 0;

    // sync output counters
    for (uint8_t i = 0; i < sync_callback_size; i++) {
        sync_callbacks[i].mod_counter = 0;
        sync_callbacks[i].tick = 0;
    }

    // stepseq counters
    for (uint8_t track=0; track < track_slots_size; track++) {
        tracks[track].step_counter = 0;
        tracks[track].mod_step_counter = 0;
    }

    // external bpm read buffer
    //for (uint8_t i=0; i < ext_interval_buffer_size; i++)
    //    ext_interval_buffer[i] = 0;
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

uint16_t uClockClass::getIntOverflowCounter()
{
    uint16_t counter = 0;
    ATOMIC(counter = int_overflow_counter)
    return counter;
}

uint16_t uClockClass::getExtOverflowCounter()
{
    uint16_t counter = 0;
    ATOMIC(counter = ext_overflow_counter)
    return counter;
}

} } // end namespace umodular::clock

umodular::clock::uClockClass uClock;
