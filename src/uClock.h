/*!
 *  @file       uClock.h
 *  Project     BPM clock generator for Arduino
 *  @brief      A Library to implement BPM clock tick calls using hardware interruption. Supported and tested on AVR boards(ATmega168/328, ATmega16u4/32u4 and ATmega2560) and ARM boards(RPI2040, Teensy, Seedstudio XIAO M0 and ESP32)
 *  @version    2.2.1
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
    int8_t step[MAX_SHUFFLE_TEMPLATE_SIZE] = {0};
} SHUFFLE_TEMPLATE;

#define MIN_BPM	1
#define MAX_BPM	400

#define PHASE_FACTOR 16
#define PLL_X 220

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
            PAUSED = 0,
            STARTING,
            SYNCING,
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

        ClockState clock_state;

        uClockClass();
        ~uClockClass();

        void setOnOutputPPQN(void (*callback)(uint32_t tick)) {
            onOutputPPQNCallback = callback;
        }

        void setOnStep(void (*callback)(uint32_t step)) {
            onStepCallback = callback;
        }

        // multiple output clock signatures
        void setOnSync1(void (*callback)(uint32_t tick)) {
            onSync1Callback = callback;
        }

        void setOnSync2(void (*callback)(uint32_t tick)) {
            onSync2Callback = callback;
        }

        void setOnSync4(void (*callback)(uint32_t tick)) {
            onSync4Callback = callback;
        }

        void setOnSync8(void (*callback)(uint32_t tick)) {
            onSync8Callback = callback;
        }

        void setOnSync12(void (*callback)(uint32_t tick)) {
            onSync12Callback = callback;
        }

        void setOnSync24(void (*callback)(uint32_t tick)) {
            onSync24Callback = callback;
        }

        void setOnSync48(void (*callback)(uint32_t tick)) {
            onSync48Callback = callback;
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

        // for software timer implementation(fallback for no board support)
        void run();

        // external timming control
        void setClockMode(ClockMode tempo_mode);
        ClockMode getClockMode();
        void clockMe();
        // for smooth slave tempo calculate display you should raise the
        // buffer_size of ext_interval_buffer in between 64 to 128. 254 max size.
        // note: this doesn't impact on sync time, only display time getTempo()
        // if you dont want to use it, it is default set it to 1 for memory save
        void setExtIntervalBuffer(uint8_t buffer_size);

        // shuffle
        void setShuffle(bool active);
        bool isShuffled();
        void setShuffleSize(uint8_t size);
        void setShuffleData(uint8_t step, int8_t tick);
        void setShuffleTemplate(int8_t * shuff, uint8_t size);
        // use this to know how many positive or negative ticks to add to current note length
        int8_t getShuffleLength();

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

        // shuffle
        bool inline processShuffle();

        void (*onOutputPPQNCallback)(uint32_t tick);
        void (*onStepCallback)(uint32_t step);
        void (*onSync1Callback)(uint32_t tick);
        void (*onSync2Callback)(uint32_t tick);
        void (*onSync4Callback)(uint32_t tick);
        void (*onSync8Callback)(uint32_t tick);
        void (*onSync12Callback)(uint32_t tick);
        void (*onSync24Callback)(uint32_t tick);
        void (*onSync48Callback)(uint32_t tick);
        void (*onClockStartCallback)();
        void (*onClockStopCallback)();
        void (*onClockPauseCallback)();
        void (*onClockContinueCallback)();

        // clock input/output control
        PPQNResolution output_ppqn = PPQN_96;
        PPQNResolution input_ppqn = PPQN_24;
        // output and internal counters, ticks and references
        uint32_t tick;
        uint32_t int_clock_tick;
        uint8_t mod_clock_counter;
        uint16_t mod_clock_ref;
        uint8_t mod_step_counter;
        uint8_t mod_step_ref;
        uint32_t step_counter;
        uint8_t mod_sync1_counter;
        uint16_t mod_sync1_ref;
        uint32_t sync1_tick;
        uint8_t mod_sync2_counter;
        uint16_t mod_sync2_ref;
        uint32_t sync2_tick;
        uint8_t mod_sync4_counter;
        uint16_t mod_sync4_ref;
        uint32_t sync4_tick;
        uint8_t mod_sync8_counter;
        uint16_t mod_sync8_ref;
        uint32_t sync8_tick;
        uint8_t mod_sync12_counter;
        uint16_t mod_sync12_ref;
        uint32_t sync12_tick;
        uint8_t mod_sync24_counter;
        uint16_t mod_sync24_ref;
        uint32_t sync24_tick;
        uint8_t mod_sync48_counter;
        uint16_t mod_sync48_ref;
        uint32_t sync48_tick;

        // external clock control
        volatile uint32_t ext_clock_us;
        volatile uint32_t ext_clock_tick;
        volatile uint32_t ext_interval;
        uint32_t last_interval;
        uint32_t sync_interval;

        volatile float tempo;
        volatile ClockMode clock_mode;
        uint32_t start_timer;

        volatile uint32_t * ext_interval_buffer = nullptr;
        uint8_t ext_interval_buffer_size;
        uint16_t ext_interval_idx;

        // shuffle implementation
        volatile SHUFFLE_TEMPLATE shuffle;
        int8_t last_shff = 0;
        bool shuffle_shoot_ctrl = true;
        volatile int8_t shuffle_length_ctrl = 0;
};

} } // end namespace umodular::clock

extern umodular::clock::uClockClass uClock;

extern "C" {
    extern volatile uint32_t _millis;
}

#endif /* __U_CLOCK_H__ */
