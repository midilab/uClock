/*!
 *  @file       uClock.cpp
 *  Project     BPM clock generator for Arduino
 *  @brief      A Library to implement BPM clock tick calls using hardware interruption. Supported and tested on AVR boards(ATmega168/328, ATmega16u4/32u4 and ATmega2560) and ARM boards(RPI2040, Teensy, Seedstudio XIAO M0 and ESP32)
 *  @version    2.3.0
 *  @author     Romulo Silva
 *  @date       10/06/2017
 *  @license    MIT - (c) 2025 - Romulo Silva - contact@midilab.co
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
    // RP2040 and RP2350 (Raspberry Pico and Pico 2) family
    //
    #if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
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
    #if !defined(USE_UCLOCK_SOFTWARE_TIMER)
        #define USE_UCLOCK_SOFTWARE_TIMER
    #endif
#endif

#if !defined(UCLOCK_HAS_PLATFORM_EXTERNAL_CLOCK_TIMESTAMP)
    static inline uint32_t uclockPlatformExternalClockTimestamp()
    {
        return micros();
    }

    static inline uint32_t uclockPlatformExternalClockIntervalUs(
        uint32_t previous_timestamp, uint32_t current_timestamp)
    {
        return current_timestamp - previous_timestamp;
    }
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

    #ifdef UCLOCK_ENABLE_TRACE
        delete[] trace_events;
    #endif
}

void uClockClass::init()
{
    if (ext_interval_buffer == nullptr)
        setExtIntervalBuffer(1);

    // initialize reference data
    calculateReferencedata();
    // initialize hardware timer
    uClockInitTimer();
    initialized = true;
    // first interval calculus
    setTempo(tempo);
}

void uClockClass::handleInternalClock()
{
    static uint32_t counter = 0;
    static uint32_t sync_interval = 0;

    // for debug usage while developing any application under uClock
    ++int_overflow_counter;

    if (int_overflow_counter > 1) {
        #ifdef UCLOCK_ENABLE_TRACE
            traceEvent(TRACE_INTERNAL_REENTRY, UINT8_MAX, 0, 0, 0, -1, int_overflow_counter);
        #endif
        --int_overflow_counter;
        return;
    }

    if (clock_state <= STARTING) { // STOPPED=0, PAUSED=1, STARTING=2, SYNCING=3, STARTED=4
        --int_overflow_counter;
        return;
    }

    if (clock_mode == EXTERNAL_CLOCK && strict_external_mode && external_ticks_remaining == 0) {
        if (
            !external_clock_stalled
            && ext_clock_us > 0
            && clock_diff(ext_clock_us, micros()) > getExternalClockStallTimeout()
        ) {
            external_clock_stalled = true;
            #ifdef UCLOCK_ENABLE_TRACE
                traceEvent(TRACE_EXTERNAL_STALLED, UINT8_MAX, 0, 0, 0, -1,
                    clock_diff(ext_clock_us, micros()));
            #endif
        }
        --int_overflow_counter;
        return;
    }

    last_internal_tick_us = micros();

    // tick phase lock and external tempo match for EXTERNAL_CLOCK mode
    if (clock_mode == EXTERNAL_CLOCK && !strict_external_mode) {
        int64_t phase_error = (int64_t)int_clock_tick - (int64_t)ext_clock_tick;

        // Tick Phase-lock
        if (!strict_external_mode && (phase_error > 1 || phase_error < -1)) {

            #ifdef UCLOCK_ENABLE_TRACE
                if (!trace_phase_error_active) {
                    traceEvent(TRACE_PHASE_ERROR, UINT8_MAX, 0, 0, 0, -1,
                            (int32_t)phase_error);
                    trace_phase_error_active = true;
                }
            #endif

            int mod_amount = 0;

            // only update tick at a full quarter or phase_lock_quarters * a quarter
            // how many quarters to count until we phase-lock?
            if (
                ((ext_clock_tick * mod_clock_ref) % (output_ppqn*phase_lock_quarters)) == mod_amount
            ) {
                #ifdef UCLOCK_ENABLE_TRACE
                    traceEvent(TRACE_PHASE_LOCK, UINT8_MAX, 0, 0, 0, -1,
                           (int32_t)phase_error);
                #endif
                tick = ext_clock_tick * mod_clock_ref;
                int_clock_tick = ext_clock_tick;
                #ifdef UCLOCK_ENABLE_TRACE
                    trace_phase_error_active = false;
                #endif
                // update any counter reference to lock with int_clock_tick
                for (uint8_t track=0; track < track_slots_size; track++) {
                    tracks[track].step_counter = tick/mod_step_ref;
                    tracks[track].mod_step_counter = 0;
                    tracks[track].shuffle.current_shff_valid = false;
                    tracks[track].shuffle.previous_shff = 0;
                    tracks[track].shuffle.skip_next_phase_zero = false;
                    tracks[track].shuffle.shuffle_shoot_ctrl = true;
                }
                // update counter reference for sync callbacks
                for (uint8_t i = 0; i < sync_callback_size; i++) {
                    if (sync_callbacks[i].callback) {
                        sync_callbacks[i].tick = tick/sync_callbacks[i].sync_ref;
                        sync_callbacks[i].mod_counter = 0;
                    }
                }
            }
        } else {
            #ifdef UCLOCK_ENABLE_TRACE
                trace_phase_error_active = false;
            #endif
        }

        // use buffer average for stable tempo estimation; raw ext_interval can be corrupted by USB bursts
        {
            uint32_t avg_interval = 0;
            uint8_t valid = 0;
            for (uint8_t i = 0; i < ext_interval_buffer_size; i++) {
                if (ext_interval_buffer[i] > 0) {
                    avg_interval += ext_interval_buffer[i];
                    valid++;
                }
            }
            if (valid > 0) {
                counter = avg_interval / valid;
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
                    #ifdef UCLOCK_ENABLE_TRACE
                        traceEvent(TRACE_TEMPO_CHANGE, UINT8_MAX, 0, 0, 0, -1,
                                (int32_t)(external_tempo * 1000.0f));
                    #endif
                    tempo = external_tempo;
                    uClockSetTimerTempo(tempo);
                }
            }
        }
    }

    // main input clock counter control
    if (mod_clock_counter >= mod_clock_ref)
        mod_clock_counter = 0;
    // process internal clock signal
    // int_clock_tick is the internal clock reference. mainly used for external clock phase lock
    if (mod_clock_counter == 0)
        ++int_clock_tick;
    ++mod_clock_counter;

    // sync callbacks
    for (uint8_t i = 0; i < sync_callback_size; i++) {
        if (sync_callbacks[i].mod_counter >= sync_callbacks[i].sync_ref)
            sync_callbacks[i].mod_counter = 0;
        if (sync_callbacks[i].mod_counter == 0) {
            if (sync_callbacks[i].callback)
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

    if (onOutputPPQNEndCallback)
        onOutputPPQNEndCallback(tick);

    // internal ticking
    ++tick;

    if (clock_mode == EXTERNAL_CLOCK && strict_external_mode &&
        external_ticks_remaining > 0)
        --external_ticks_remaining;

    if (clock_mode == INTERNAL_CLOCK) {
        uint32_t base_interval = bpmToMicroSeconds(tempo);
        if (internal_phase_slew_ticks_remaining > 0) {
            int32_t adjustment = internal_phase_correction_remaining_us /
                (int32_t)internal_phase_slew_ticks_remaining;
            if (adjustment == 0 && internal_phase_correction_remaining_us != 0)
                adjustment = internal_phase_correction_remaining_us > 0 ? 1 : -1;

            int32_t adjusted_interval = (int32_t)base_interval + adjustment;
            int32_t minimum_interval = (int32_t)base_interval / 2;
            int32_t maximum_interval = (int32_t)base_interval +
                (int32_t)base_interval / 2;
            if (adjusted_interval < minimum_interval)
                adjusted_interval = minimum_interval;
            else if (adjusted_interval > maximum_interval)
                adjusted_interval = maximum_interval;

            adjustment = adjusted_interval - (int32_t)base_interval;
            internal_phase_correction_remaining_us -= adjustment;
            --internal_phase_slew_ticks_remaining;
            setTimer((uint32_t)adjusted_interval);
            internal_phase_timer_adjusted = true;
        } else if (internal_phase_timer_adjusted) {
            setTimer(base_interval);
            internal_phase_timer_adjusted = false;
        }
    }

    // for debug usage while developing any application under uClock
    --int_overflow_counter;
}

void uClockClass::handleExternalClock()
{
    handleExternalClock(micros(), uclockPlatformExternalClockTimestamp(), true);
}

void uClockClass::handleExternalClock(uint32_t observed_at_us,
                                      uint32_t platform_timestamp,
                                      bool platform_timestamp_valid)
{
    static uint8_t start_sync_counter = 0;
    #if defined(UCLOCK_ENABLE_TRACE) && defined(UCLOCK_TRACE_EXTERNAL_CLOCK_TIMING)
        int32_t external_timing_delta = 0;
        bool external_timing_delta_valid = false;
    #endif

    // for debug usage while developing any application under uClock
    ++ext_overflow_counter;

    #ifdef UCLOCK_ENABLE_TRACE
        if (ext_overflow_counter > 1)
            traceEvent(TRACE_EXTERNAL_REENTRY, UINT8_MAX, 0, 0, 0, -1, ext_overflow_counter);
    #endif

    // calculate and store ext_interval
    if (ext_clock_us > 0) {
        uint32_t wall_interval = clock_diff(ext_clock_us, observed_at_us);
        uint32_t platform_interval = wall_interval;
        if (ext_clock_timestamp_valid && platform_timestamp_valid) {
            platform_interval = uclockPlatformExternalClockIntervalUs(
                ext_clock_timestamp, platform_timestamp);
        }
        #if defined(UCLOCK_ENABLE_TRACE) && defined(UCLOCK_TRACE_EXTERNAL_CLOCK_TIMING)
            external_timing_delta = (int32_t)wall_interval - (int32_t)platform_interval;
            external_timing_delta_valid = ext_clock_timestamp_valid &&
                platform_timestamp_valid;
        #endif
        #if defined(UCLOCK_EXTERNAL_CLOCK_USE_MICROS)
            ext_interval = wall_interval;
        #else
        ext_interval = ext_clock_timestamp_valid && platform_timestamp_valid &&
            wall_interval <= getExternalClockStallTimeout()
            ? platform_interval
            : wall_interval;
        #endif
    }
    ext_clock_us = observed_at_us;
    ext_clock_timestamp = platform_timestamp;
    ext_clock_timestamp_valid = platform_timestamp_valid;

    // external clock tick me!
    ext_clock_tick++;

    #ifdef UCLOCK_ENABLE_TRACE
        traceEvent(TRACE_EXTERNAL_PULSE, UINT8_MAX, 0, 0, 0, -1, ext_interval);
        #if defined(UCLOCK_TRACE_EXTERNAL_CLOCK_TIMING)
            if (external_timing_delta_valid)
                traceEvent(TRACE_EXTERNAL_TIMING_DELTA, UINT8_MAX, 0, 0, 0, -1,
                        external_timing_delta);
        #endif
    #endif

    if (strict_external_mode &&
        (clock_state == STARTING || clock_state == SYNCING || clock_state == STARTED)) {
        bool discontinuity = last_accepted_external_interval > 0 &&
            ext_interval > getExternalClockStallTimeout();
        if (discontinuity) {
            for (uint8_t i = 0; i < ext_interval_buffer_size; i++)
                ext_interval_buffer[i] = 0;
            ext_interval_idx = 0;
            if (!external_clock_stalled) {
                external_clock_stalled = true;
                #ifdef UCLOCK_ENABLE_TRACE
                    traceEvent(TRACE_EXTERNAL_STALLED, UINT8_MAX, 0, 0, 0, -1,
                           ext_interval);
                #endif
            }
        }

        if (external_clock_stalled) {
            external_clock_stalled = false;
            #ifdef UCLOCK_ENABLE_TRACE
                traceEvent(TRACE_EXTERNAL_RESUMED, UINT8_MAX, 0, 0, 0, -1,
                       ext_interval);
            #endif
        }

        if (!discontinuity)
            updateExternalTempo(ext_interval);

        if (clock_state != STARTED)
            clock_state = STARTED;

        uint16_t low_ppqn_target_ticks = mod_clock_ref;
        if (input_ppqn < PPQN_24 && mod_clock_ref > 0) {
            uint16_t phase = tick % mod_clock_ref;
            if (phase > 0) {
                low_ppqn_target_ticks = phase <= mod_clock_ref / 2
                    ? mod_clock_ref - phase
                    : (mod_clock_ref * 2) - phase;
                #ifdef UCLOCK_ENABLE_TRACE
                    traceEvent(TRACE_PHASE_LOCK, UINT8_MAX, 0, 0, 0, -1,
                           (int32_t)low_ppqn_target_ticks - mod_clock_ref);
                #endif
            }

            uint32_t minimum_interval = 60000000UL / input_ppqn / MAX_BPM;
            uint32_t maximum_interval = 60000000UL / input_ppqn / MIN_BPM;
            uint32_t input_interval = ext_interval >= minimum_interval &&
                ext_interval <= maximum_interval
                ? ext_interval
                : bpmToMicroSeconds(tempo) * mod_clock_ref;
            setTimer(input_interval / low_ppqn_target_ticks);
        }

        #ifdef UCLOCK_ENABLE_TRACE
            uint16_t pending_count = external_ticks_remaining;
        #endif
        ATOMIC(
            if (input_ppqn < PPQN_24) {
                external_ticks_remaining = low_ppqn_target_ticks;
            } else {
                uint32_t authorized_ticks = (uint32_t)external_ticks_remaining +
                    mod_clock_ref;
                external_ticks_remaining = authorized_ticks > UINT16_MAX
                    ? UINT16_MAX
                    : (uint16_t)authorized_ticks;
            }
        )
        #ifdef UCLOCK_ENABLE_TRACE
            if (pending_count > 0)
                traceEvent(TRACE_EXTERNAL_CATCH_UP, UINT8_MAX, 0, 0, 0, -1,
                        pending_count);
        #endif
        handleInternalClock();

        --ext_overflow_counter;
        return;
    }

    switch (clock_state) {
        case STARTING:
            clock_state = SYNCING;
            start_sync_counter = MINIMUM_SYNC_COUNTER;
            handleInternalClock();
            break;
        case SYNCING:
            // Accumulate valid intervals during SYNCING so the PLL buffer has real
            // data by the time we reach STARTED.
            if (ext_interval >= (60000000UL / input_ppqn / MAX_BPM)) {
                ext_interval_buffer[ext_interval_idx] = ext_interval;
                if (++ext_interval_idx >= ext_interval_buffer_size)
                    ext_interval_idx = 0;
            }
            if (--start_sync_counter == 0) {
                // Force-align all internal counters to ext_clock_tick, which is
                // always the canonical song position.  The existing phase-lock only
                // snaps on beat boundaries, which means without this we can start
                // up to a full beat out of alignment.
                tick = ext_clock_tick * mod_clock_ref;
                int_clock_tick = ext_clock_tick;
                mod_clock_counter = 0;
                for (uint8_t track = 0; track < track_slots_size; track++) {
                    tracks[track].step_counter = tick / mod_step_ref;
                    tracks[track].mod_step_counter = 0;
                    tracks[track].shuffle.current_shff_valid = false;
                    tracks[track].shuffle.previous_shff = 0;
                    tracks[track].shuffle.skip_next_phase_zero = false;
                    tracks[track].shuffle.shuffle_shoot_ctrl = true;
                }
                for (uint8_t i = 0; i < sync_callback_size; i++) {
                    if (sync_callbacks[i].callback) {
                        sync_callbacks[i].tick = tick / sync_callbacks[i].sync_ref;
                        sync_callbacks[i].mod_counter = 0;
                    }
                }
                // Prime the timer to the correct BPM immediately.
                if (ext_interval >= (60000000UL / input_ppqn / MAX_BPM)) {
                    tempo = constrainBpm(freqToBpm(ext_interval));
                    uClockSetTimerTempo(tempo);
                }
                clock_state = STARTED;
            }
            break;
        default:
            // accumulate interval incoming ticks data for getTempo() smooth reads on slave clock_mode
            // reject intervals shorter than the minimum valid period at MAX_BPM (filters USB burst packets)
            if (ext_interval >= (60000000UL / input_ppqn / MAX_BPM)) {
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

void uClockClass::clockMeAt(uint32_t observed_at_us)
{
    ATOMIC(handleExternalClock(observed_at_us, 0, false))
}

void uClockClass::setStrictExternalMode(bool strict) 
{
    strict_external_mode = strict;
}
bool uClockClass::isStrictExternalMode() 
{
    return strict_external_mode;
}

uint32_t uClockClass::getExternalClockStallTimeout()
{
    if (last_accepted_external_interval == 0)
        return 250000;

    uint32_t timeout = last_accepted_external_interval * 4;
    return timeout < 20000 ? 20000 : timeout;
}

void uClockClass::updateExternalTempo(uint32_t interval)
{
    uint32_t minimum_interval = 60000000UL / input_ppqn / MAX_BPM;
    uint32_t maximum_interval = 60000000UL / input_ppqn / MIN_BPM;
    if (interval < minimum_interval || interval > maximum_interval)
        return;

    last_accepted_external_interval = interval;
    ext_interval_buffer[ext_interval_idx] = interval;
    if (++ext_interval_idx >= ext_interval_buffer_size)
        ext_interval_idx = 0;

    uint8_t tempo_window_size = input_ppqn < ext_interval_buffer_size
        ? input_ppqn
        : ext_interval_buffer_size;
    uint64_t total = 0;
    uint8_t count = 0;
    uint32_t minimum = UINT32_MAX;
    uint32_t maximum = 0;
    for (uint8_t offset = 0; offset < tempo_window_size; offset++) {
        uint8_t index = (ext_interval_idx + ext_interval_buffer_size - 1 - offset) %
            ext_interval_buffer_size;
        uint32_t sample = ext_interval_buffer[index];
        if (sample > 0) {
            total += sample;
            if (sample < minimum)
                minimum = sample;
            if (sample > maximum)
                maximum = sample;
            ++count;
        }
    }
    if (count == 0)
        return;

    if (count >= 3) {
        total -= minimum;
        total -= maximum;
        count -= 2;
    }
    external_tempo = constrainBpm(freqToBpm(total / count));
    if (external_tempo != tempo) {
        #ifdef UCLOCK_ENABLE_TRACE
            traceEvent(TRACE_TEMPO_CHANGE, UINT8_MAX, 0, 0, 0, -1,
                    (int32_t)(external_tempo * 1000.0f));
        #endif
        tempo = external_tempo;
        uClockSetTimerTempo(tempo);
    }
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

    #ifdef UCLOCK_ENABLE_TRACE
        traceEvent(TRACE_START);
    #endif

    if (onClockStartCallback)
        onClockStartCallback();
}

void uClockClass::stop()
{
    ATOMIC(clock_state = STOPPED)
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
        if (tempo_mode == EXTERNAL_CLOCK && clock_mode != EXTERNAL_CLOCK) {
            ext_clock_us = 0;
            ext_clock_timestamp = 0;
            ext_clock_timestamp_valid = false;
            ext_interval = 0;
            last_accepted_external_interval = 0;
            external_ticks_remaining = 0;
            external_clock_stalled = false;
            ext_interval_idx = 0;
            if (ext_interval_buffer != nullptr) {
                for (uint8_t i = 0; i < ext_interval_buffer_size; i++)
                    ext_interval_buffer[i] = 0;
            }
        }
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
    #if defined(USE_UCLOCK_SOFTWARE_TIMER)
        // call software timer implementation
        softwareTimerHandler(micros());
    #endif
}

void uClockClass::stepSeqTick()
{
    for (uint8_t track=0; track < track_slots_size; track++) {
        bool stepProcess = false;
        if (tracks[track].mod_step_counter >= mod_step_ref)
            tracks[track].mod_step_counter = 0;
        if (!tracks[track].shuffle.tmplt.active) {
            if (tracks[track].mod_step_counter == 0
                && int32_t(uint32_t(tracks[track].step_counter * mod_step_ref) - tick) <= 0)
                stepProcess = true;
        } else if (processShuffle(track)) {
            stepProcess = true;
        }

        if (stepProcess) {
            #ifdef UCLOCK_ENABLE_TRACE
                int8_t shuffle_value = 0;
                int16_t shuffle_target = 0;
                if (tracks[track].shuffle.tmplt.active) {
                    shuffle_value = tracks[track].shuffle.current_shff;
                    shuffle_target = shuffle_value >= 0
                        ? shuffle_value
                        : mod_step_ref + shuffle_value;
                }
                traceEvent(TRACE_STEP_FIRE, track, tracks[track].step_counter,
                        tracks[track].mod_step_counter, shuffle_value, shuffle_target);
            #endif
            if (onStepGlobalCallback)
                onStepGlobalCallback(tracks[track].step_counter);
            if (onStepMultiCallback)
                onStepMultiCallback(tracks[track].step_counter, track);

            // going forward to the next step call
            ++tracks[track].step_counter;
            tracks[track].shuffle.current_shff_valid = false;
        }
        ++tracks[track].mod_step_counter;
    }

    #ifdef UCLOCK_ENABLE_TRACE
        if (track_slots_size > 1) {
            for (uint8_t track = 1; track < track_slots_size; track++) {
                if (tracks[track].mod_step_counter != tracks[0].mod_step_counter) {
                    traceEvent(TRACE_STEP_PHASE_DIVERGED, track, tracks[track].step_counter,
                            tracks[track].mod_step_counter, 0, tracks[0].mod_step_counter,
                            (int32_t)tracks[track].mod_step_counter - tracks[0].mod_step_counter);
                }
            }
        }
    #endif
}

void uClockClass::setShuffle(bool active, uint8_t track)
{
    if (tracks == nullptr || track >= track_slots_size)
        return;

    ATOMIC(
        if (tracks[track].shuffle.tmplt.active != active) {
            #ifdef UCLOCK_ENABLE_TRACE
                bool previous = tracks[track].shuffle.tmplt.active;
            #endif
            tracks[track].shuffle.tmplt.active = active;
            tracks[track].shuffle.current_shff_valid = false;
            tracks[track].shuffle.previous_shff = 0;
            tracks[track].shuffle.skip_next_phase_zero = false;
            // A newly enabled nonnegative step belongs to the next modulo
            // window. processShuffle() immediately arms negative offsets.
            tracks[track].shuffle.shuffle_shoot_ctrl = !active;
            if (active && mod_step_ref > 0) {
                const uint32_t nominal_tick = tracks[track].step_counter * mod_step_ref;
                const int32_t until_nominal = int32_t(nominal_tick - tick);
                tracks[track].shuffle.skip_next_phase_zero =
                    until_nominal >= mod_step_ref;
                const int8_t offset = tracks[track].shuffle.tmplt.step[
                    tracks[track].step_counter % tracks[track].shuffle.tmplt.size];
                if (offset < 0 && until_nominal + offset < 0) {
                    tracks[track].shuffle.current_shff = 0;
                    tracks[track].shuffle.current_shff_valid = true;
                }
            }
            #ifdef UCLOCK_ENABLE_TRACE
                traceEvent(TRACE_SHUFFLE_STATE, track, tracks[track].step_counter,
                       tracks[track].mod_step_counter, active ? 1 : 0, -1,
                       previous ? 1 : 0);
            #endif
        }
    )
}

bool uClockClass::isShuffled(uint8_t track)
{
    if (tracks == nullptr || track >= track_slots_size)
        return false;

    return tracks[track].shuffle.tmplt.active;
}

void uClockClass::setShuffleSize(uint8_t size, uint8_t track)
{
    if (tracks == nullptr || track >= track_slots_size || size == 0)
        return;

    if (size > MAX_SHUFFLE_TEMPLATE_SIZE)
        size = MAX_SHUFFLE_TEMPLATE_SIZE;
    ATOMIC(tracks[track].shuffle.tmplt.size = size)
}

void uClockClass::setShuffleData(uint8_t step, int8_t tick, uint8_t track)
{
    if (tracks == nullptr || track >= track_slots_size)
        return;

    if (step >= MAX_SHUFFLE_TEMPLATE_SIZE ||
        tick <= -(int16_t)mod_step_ref || tick >= (int16_t)mod_step_ref)
        return;
        ATOMIC(
            #ifdef UCLOCK_ENABLE_TRACE
                int8_t previous = tracks[track].shuffle.tmplt.step[step];
            #endif
            tracks[track].shuffle.tmplt.step[step] = tick;
            #ifdef UCLOCK_ENABLE_TRACE
                if (previous != tick)
                    traceEvent(TRACE_SHUFFLE_CHANGE, track, step,
                        tracks[track].mod_step_counter, tick, -1, previous);
            #endif
        )
}

void uClockClass::setShuffleTemplate(const int8_t * shuff, uint8_t size, uint8_t track)
{
    if (tracks == nullptr || track >= track_slots_size || shuff == nullptr || size == 0)
        return;

    if (size > MAX_SHUFFLE_TEMPLATE_SIZE)
        size = MAX_SHUFFLE_TEMPLATE_SIZE;
    for (uint8_t i = 0; i < size; i++) {
        if (shuff[i] <= -(int16_t)mod_step_ref || shuff[i] >= (int16_t)mod_step_ref)
            return;
    }

    ATOMIC(
        tracks[track].shuffle.tmplt.size = size;
        for (uint8_t i = 0; i < size; i++) {
            #ifdef UCLOCK_ENABLE_TRACE
                int8_t previous = tracks[track].shuffle.tmplt.step[i];
            #endif
            tracks[track].shuffle.tmplt.step[i] = shuff[i];
            #ifdef UCLOCK_ENABLE_TRACE
                if (previous != shuff[i])
                    traceEvent(TRACE_SHUFFLE_CHANGE, track, i,
                            tracks[track].mod_step_counter, shuff[i], -1, previous);
            #endif
        }
    )
}

int8_t uClockClass::getShuffleLength(uint8_t track)
{
    if (tracks == nullptr || track >= track_slots_size)
        return 0;

    return tracks[track].shuffle.shuffle_length_ctrl;
}

int8_t uClockClass::getShuffleOffset(uint32_t step, uint8_t track)
{
    if (tracks == nullptr || track >= track_slots_size || tracks[track].shuffle.tmplt.size == 0)
        return 0;

    return tracks[track].shuffle.tmplt.step[step % tracks[track].shuffle.tmplt.size];
}

bool inline uClockClass::processShuffle(uint8_t track)
{
    if (tracks == nullptr)
        return false;

    if (tracks[track].shuffle.tmplt.size == 0) {
        #ifdef UCLOCK_ENABLE_TRACE
            traceEvent(TRACE_INVALID_STATE, track, tracks[track].step_counter,
                   tracks[track].mod_step_counter, 0, -1, 1);
        #endif
        return false;
    }

    int8_t mod_shuffle = 0;

    if (!tracks[track].shuffle.current_shff_valid) {
        tracks[track].shuffle.current_shff = tracks[track].shuffle.tmplt.step[
            tracks[track].step_counter % tracks[track].shuffle.tmplt.size];
        tracks[track].shuffle.current_shff_valid = true;
        if (tracks[track].shuffle.current_shff < 0) {
            tracks[track].shuffle.shuffle_shoot_ctrl = true;
        } else if (tracks[track].shuffle.previous_shff < 0) {
            tracks[track].shuffle.skip_next_phase_zero = true;
        }
    }

    if (!tracks[track].shuffle.shuffle_shoot_ctrl &&
        tracks[track].mod_step_counter == 0) {
        if (tracks[track].shuffle.skip_next_phase_zero) {
            tracks[track].shuffle.skip_next_phase_zero = false;
        } else {
            tracks[track].shuffle.shuffle_shoot_ctrl = true;
        }
    }

    int8_t shff = tracks[track].shuffle.current_shff;

    if (shff >= 0) {
        mod_shuffle = tracks[track].mod_step_counter - shff;
    } else if (shff < 0) {
        mod_shuffle = tracks[track].mod_step_counter - (mod_step_ref + shff);
    }

    // shuffle_shoot_ctrl helps keep track if we have shoot or not a note for the step space of output_ppqn/4 pulses
    if (mod_shuffle == 0 && tracks[track].shuffle.shuffle_shoot_ctrl == true) {
        // keep track of next note shuffle for current note length control
        tracks[track].shuffle.shuffle_length_ctrl = tracks[track].shuffle.tmplt.step[(tracks[track].step_counter+1)%tracks[track].shuffle.tmplt.size];
        tracks[track].shuffle.shuffle_length_ctrl -= shff;
        tracks[track].shuffle.previous_shff = shff;
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

void uClockClass::setOnSync(PPQNResolution resolution, void (*callback)(uint32_t tick)) {
    // Callback storage is immutable once the timer can access it.
    if (initialized || resolution > output_ppqn || callback == nullptr)
        return;

    SyncCallback * new_sync_callbacks = new SyncCallback[sync_callback_size + 1];
    if (sync_callbacks != nullptr) {
        memcpy(new_sync_callbacks, sync_callbacks,
               sizeof(SyncCallback) * sync_callback_size);
        delete[] sync_callbacks;
    }
    new_sync_callbacks[sync_callback_size].callback = callback;
    new_sync_callbacks[sync_callback_size].resolution = resolution;
    new_sync_callbacks[sync_callback_size].sync_ref = output_ppqn / resolution;

    sync_callbacks = new_sync_callbacks;
    ++sync_callback_size;
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

bool uClockClass::syncInternalClockToBeat(uint32_t observed_at_us)
{
    if (clock_mode != INTERNAL_CLOCK || clock_state != STARTED ||
        last_internal_tick_us == 0 || output_ppqn == 0)
        return false;

    uint32_t tick_interval = bpmToMicroSeconds(tempo);
    uint32_t beat_interval = tick_interval * output_ppqn;
    uint32_t last_tick = tick > 0 ? tick - 1 : 0;
    uint32_t elapsed = clock_diff(last_internal_tick_us, observed_at_us);
    uint32_t phase_us = ((last_tick % output_ppqn) * tick_interval + elapsed) %
        beat_interval;
    int32_t correction = phase_us <= beat_interval / 2
        ? (int32_t)phase_us
        : (int32_t)phase_us - (int32_t)beat_interval;

    ATOMIC(
        internal_phase_correction_us = correction;
        internal_phase_correction_remaining_us = correction;
        internal_phase_slew_ticks_remaining = output_ppqn;
    )
    return true;
}

int32_t uClockClass::getInternalPhaseCorrectionUs()
{
    int32_t correction = 0;
    ATOMIC(correction = internal_phase_correction_us)
    return correction;
}

uint16_t uClockClass::getInternalPhaseSlewTicksRemaining()
{
    uint16_t remaining = 0;
    ATOMIC(remaining = internal_phase_slew_ticks_remaining)
    return remaining;
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
    if (count == 0)
        count = 1;
    ATOMIC(phase_lock_quarters = count)
}

void uClockClass::resetCounters()
{
    tick = 0;
    mod_clock_counter = 0;
    int_clock_tick = 0;
    ext_clock_tick = 0;
    ext_clock_us = 0;
    ext_clock_timestamp = 0;
    ext_clock_timestamp_valid = false;
    ext_interval = 0;
    external_ticks_remaining = 0;
    external_clock_stalled = false;
    last_accepted_external_interval = 0;
    last_internal_tick_us = 0;
    internal_phase_correction_us = 0;
    internal_phase_correction_remaining_us = 0;
    internal_phase_slew_ticks_remaining = 0;
    internal_phase_timer_adjusted = false;
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
        tracks[track].shuffle.current_shff_valid = false;
        tracks[track].shuffle.previous_shff = 0;
        tracks[track].shuffle.skip_next_phase_zero = false;
        tracks[track].shuffle.shuffle_shoot_ctrl = true;
    }

    // external bpm read buffer
    for (uint8_t i=0; i < ext_interval_buffer_size; i++)
       ext_interval_buffer[i] = 0;
}

#ifdef UCLOCK_ENABLE_TRACE
    void uClockClass::traceEvent(TraceEventType type, uint8_t track, uint32_t step,
                                uint16_t step_phase, int8_t shuffle_value,
                                int16_t shuffle_target, int32_t value)
    {
        if (trace_frozen) {
            ++trace_dropped;
            return;
        }

        uint16_t head = trace_head;
        TraceEvent &event = trace_events[head];
        event.timestamp_us = micros();
        event.tick = tick;
        event.int_clock_tick = int_clock_tick;
        event.ext_clock_tick = ext_clock_tick;
        event.step = step;
        event.value = value;
        event.mod_clock_counter = mod_clock_counter;
        event.mod_step_counter = step_phase;
        event.shuffle_target = shuffle_target;
        event.shuffle_value = shuffle_value;
        event.track = track;
        event.type = type;
        event.clock_state = clock_state;
        event.handler_depth = int_overflow_counter > ext_overflow_counter
            ? int_overflow_counter
            : ext_overflow_counter;

        uint16_t next = (head + 1) % UCLOCK_TRACE_BUFFER_SIZE;
        if (next == trace_tail) {
            trace_tail = (trace_tail + 1) % UCLOCK_TRACE_BUFFER_SIZE;
            ++trace_dropped;
        }
        trace_head = next;

        if (type == TRACE_STEP_PHASE_DIVERGED || type == TRACE_EXTERNAL_REENTRY ||
            type == TRACE_INVALID_STATE)
            trace_frozen = true;
    }

    bool uClockClass::popTraceEvent(TraceEvent &event)
    {
        bool available = false;
        ATOMIC(
            if (trace_tail != trace_head) {
                event = trace_events[trace_tail];
                trace_tail = (trace_tail + 1) % UCLOCK_TRACE_BUFFER_SIZE;
                available = true;
            }
        )
        return available;
    }

    void uClockClass::clearTrace()
    {
        ATOMIC(
            trace_head = 0;
            trace_tail = 0;
            trace_dropped = 0;
            trace_frozen = false;
        )
    }

    uint32_t uClockClass::getTraceDroppedCount()
    {
        uint32_t dropped = 0;
        ATOMIC(dropped = trace_dropped)
        return dropped;
    }

    bool uClockClass::isTraceFrozen()
    {
        bool frozen = false;
        ATOMIC(frozen = trace_frozen)
        return frozen;
    }
#endif

void uClockClass::tap()
{
    // we can make use of mod_sync1_ref for tap
    //uint8_t mod_tap_ref = output_ppqn / PPQN_1;
    // we only set tap if ClockMode is INTERNAL_CLOCK

    // @@TODO: this can probably be replaced by syncInternalClockToBeat now?
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

bool uClockClass::isExternalClockStalled()
{
    return external_clock_stalled;
}

uint16_t uClockClass::getExternalTicksRemaining()
{
    return external_ticks_remaining;
}

uint32_t uClockClass::getLastAcceptedExternalInterval()
{
    return last_accepted_external_interval;
}

uint32_t uClockClass::getExternalClockPulseAge()
{
    return ext_clock_us == 0 ? 0 : clock_diff(ext_clock_us, micros());
}

} } // end namespace umodular::clock

umodular::clock::uClockClass uClock;
