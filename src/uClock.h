/*!
 *  @file       uClock.h
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

#ifndef __U_CLOCK_H__
#define __U_CLOCK_H__

#include <Arduino.h>
#include <inttypes.h>

namespace umodular { namespace clock {

// Shuffle templates are specific for each PPQN output resolution
// min: -(output_ppqn/4)-1 ticks
// max: (output_ppqn/4)-1 ticks
// adjust the size of you template if more than 16 shuffle step info needed
#define MAX_SHUFFLE_TEMPLATE_SIZE   16
typedef struct {
    bool active = false;
    uint8_t size = MAX_SHUFFLE_TEMPLATE_SIZE;
    int8_t step[MAX_SHUFFLE_TEMPLATE_SIZE] = {0}; // int8 supports max PPQN_480 of internal clock resolution
} SHUFFLE_TEMPLATE;

typedef struct {
    volatile SHUFFLE_TEMPLATE tmplt;
    int8_t last_shff = 0; // int8 supports max PPQN_480 of internal clock resolution
    bool shuffle_shoot_ctrl = true;
    volatile int8_t shuffle_length_ctrl = 0;
} SHUFFLE_DATA;

typedef struct {
    SHUFFLE_DATA shuffle;
    //int8_t shift = 0;
    //uint8_t direction = 0;
    uint32_t step_counter = 0;
    uint8_t mod_step_counter;
} TRACK_SLOT;

enum SyncResolution {
    SYNC1 = 1,
    SYNC2 = 2,
    SYNC4 = 4,
    SYNC8 = 8,
    SYNC12 = 12,
    SYNC24 = 24,
    SYNC48 = 48
};

// Sync callback structure for batch processing
struct SyncCallback {
    void (*callback)(uint32_t tick) = nullptr;
    uint8_t mod_counter = 0;
    uint16_t sync_ref = 0;
    uint32_t tick = 0;
    SyncResolution resolution;
};

#define MIN_BPM	1
#define MAX_BPM	400

#define PHASE_FACTOR 16
#define PLL_X 220

#define MICROS_PER_MIN (60000000UL)
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

class uClockClass {

    public:
        enum ClockMode {
            INTERNAL_CLOCK = 0,
            EXTERNAL_CLOCK
        };

        enum ClockState {
            STOPED = 0,
            PAUSED,
            STARTING,
            STARTED
        };

        enum PPQNResolution {
            PPQN_1 = 1,
            PPQN_2 = 2,
            PPQN_4 = 4,
            PPQN_8 = 8,
            PPQN_12 = 12,
            PPQN_24 = 24,
            PPQN_48 = 48,
            PPQN_96 = 96,
            PPQN_384 = 384,
            PPQN_480 = 480,
            PPQN_960 = 960
        };

        ClockState clock_state = STOPED;

        uClockClass();
        ~uClockClass();

        void setOnOutputPPQN(void (*callback)(uint32_t tick)) {
            onOutputPPQNCallback = callback;
        }

        // multiple output sync clock signatures support
        void setOnSync(SyncResolution resolution, void (*callback)(uint32_t tick)) {
            // alloc once and forever policy!
           	// reallocate by creating a new array, copying data, and deleting the old one
           	SyncCallback * new_sync_callbacks = new SyncCallback[sync_callback_size+1];
           	if (sync_callbacks != nullptr) {
          		memcpy(new_sync_callbacks, sync_callbacks, sizeof(SyncCallback) * (sync_callback_size+1));
          		delete[] sync_callbacks;
           	}
            sync_callbacks = new_sync_callbacks;

            sync_callbacks[sync_callback_size].callback = callback;
            sync_callbacks[sync_callback_size].resolution = resolution;

            ++sync_callback_size;
        }

        void setOnSync1(void (*callback)(uint32_t tick)) {
            setOnSync(SYNC1, callback);
        }

        void setOnSync2(void (*callback)(uint32_t tick)) {
            setOnSync(SYNC2, callback);
        }

        void setOnSync4(void (*callback)(uint32_t tick)) {
            setOnSync(SYNC4, callback);
        }

        void setOnSync8(void (*callback)(uint32_t tick)) {
            setOnSync(SYNC8, callback);
        }

        void setOnSync12(void (*callback)(uint32_t tick)) {
            setOnSync(SYNC12, callback);
        }

        void setOnSync24(void (*callback)(uint32_t tick)) {
            setOnSync(SYNC24, callback);
        }

        void setOnSync48(void (*callback)(uint32_t tick)) {
            setOnSync(SYNC48, callback);
        }

        void setOnClockStart(void (*callback)()) {
            onClockStartCallback = callback;
        }

        void setOnClockStop(void (*callback)()) {
            onClockStopCallback = callback;
        }

        void setOnClockPause(void (*callback)()) {
            onClockPauseCallback = callback;
        }

        void setOnClockContinue(void (*callback)()) {
            onClockContinueCallback = callback;
        }

        // Step Seq extension support?
        // Step Seq supports per track control:
        // shuffle: YES
        // shift: ROADMAP
        // direction: ROADMAP
        // keep API compatibility for setOnStep global track
        void setOnStep(void (*callback)(uint32_t step)) {
            if (tracks == nullptr) {
                track_slots_size = 1;
                // alloc once and forever policy
                tracks = new TRACK_SLOT[track_slots_size];
                onStepGlobalCallback = callback;
            }
        }
        // extended stepSeq multitrack support for setOnStep
        void setOnStep(void (*callback)(uint32_t step, uint8_t track), uint8_t track_number) {
            if (tracks == nullptr) {
                track_slots_size = track_number;
                // alloc once and forever policy
                tracks = new TRACK_SLOT[track_slots_size];
                onStepMultiCallback = callback;
            }
        }

        void init();
        void setOutputPPQN(PPQNResolution resolution);
        void setInputPPQN(PPQNResolution resolution);

        void handleInternalClock();
        void handleExternalClock();
        void resetCounters();

        // external class control
        void start();
        void stop();
        void pause();
        void setTempo(float bpm);
        float getTempo();

        // Step Seq extension for global and multi track sequences control
        // void setShift(int8_t shift, uint8_t track = 0);
        // void setDirection(uint8_t direction, uint8_t track = 0);
        void setShuffle(bool active, uint8_t track = 0);
        bool isShuffled(uint8_t track = 0);
        void setShuffleSize(uint8_t size, uint8_t track = 0);
        void setShuffleData(uint8_t step, int8_t tick, uint8_t track = 0);
        void setShuffleTemplate(int8_t * shuff, uint8_t size, uint8_t track = 0);
        // use this to know how many positive or negative ticks to add to current note length
        int8_t getShuffleLength(uint8_t track = 0);

        // for software timer implementation(fallback for no board support)
        void run();

        // external timing control
        void setClockMode(ClockMode tempo_mode);
        ClockMode getClockMode();
        void clockMe();

        // strict external clock mode functions
        bool allowTick();
        void setStrictExternalMode(bool strict);
        bool isStrictExternalMode();
        void setPhaseLockQuartersCount(uint8_t count);
        // for smooth slave tempo calculate display you should raise the
        // buffer_size of ext_interval_buffer in between 64 to 128. 254 max size.
        // note: this doesn't impact on sync time, only display time getTempo()
        // if you dont want to use it, it is default set it to 1 for memory save
        void setExtIntervalBuffer(size_t buffer_size);

        // todo!
        void tap();

        // elapsed time support
        uint8_t getNumberOfSeconds(uint32_t time);
        uint8_t getNumberOfMinutes(uint32_t time);
        uint8_t getNumberOfHours(uint32_t time);
        uint8_t getNumberOfDays(uint32_t time);
        uint32_t getNowTimer();
        uint32_t getPlayTime();

        uint32_t bpmToMicroSeconds(float bpm);

    private:
        float inline freqToBpm(uint32_t freq);
        float inline constrainBpm(float bpm);
        void calculateReferencedata();

        // callbacks
        void (*onOutputPPQNCallback)(uint32_t tick) = nullptr;
        void (*onClockStartCallback)() = nullptr;
        void (*onClockStopCallback)() = nullptr;
        void (*onClockPauseCallback)() = nullptr;
        void (*onClockContinueCallback)() = nullptr;
        // step seq extension for global and multi track sequences control
        void (*onStepGlobalCallback)(uint32_t step) = nullptr;
        void (*onStepMultiCallback)(uint32_t step, uint8_t track) = nullptr;
        // sync callback data
        SyncCallback * sync_callbacks = nullptr;
        uint8_t sync_callback_size = 0;

        // clock core
        // input/output tick resolution
        PPQNResolution output_ppqn = PPQN_96;
        PPQNResolution input_ppqn = PPQN_24;
        volatile float tempo = 120.0;
        volatile ClockMode clock_mode = INTERNAL_CLOCK;
        uint32_t start_timer = 0;
        bool strict_external_mode = false;

        // output and internal counters, ticks and references
        volatile uint32_t tick;
        volatile uint32_t int_clock_tick;
        uint8_t mod_step_ref;
        uint8_t mod_clock_counter;
        uint16_t mod_clock_ref;

        // external clock control
        volatile uint32_t ext_clock_us = 0;
        volatile uint32_t ext_clock_tick = 0;
        volatile uint32_t ext_interval = 0;
        volatile uint32_t ext_last_interval = 0;
        volatile uint32_t request_sync = 0;
        volatile float external_tempo = tempo;
        uint8_t phase_lock_quarters = 1;

        // StepSeq extension
        // main stepseq tick processor
        void stepSeqTick();
        // stepseq shuffle processor
        bool inline processShuffle(uint8_t track = 0);
        TRACK_SLOT * tracks = nullptr;
        size_t track_slots_size = 0;

        // external clock bpm calculus
        volatile uint32_t * ext_interval_buffer = nullptr;
        size_t ext_interval_buffer_size = 0;
        uint16_t ext_interval_idx = 0;
};

} } // end namespace umodular::clock

extern umodular::clock::uClockClass uClock;

extern "C" {
    extern volatile uint32_t _millis;
}

#endif /* __U_CLOCK_H__ */
