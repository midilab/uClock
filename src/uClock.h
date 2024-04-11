/*!
 *  @file       uClock.h
 *  Project     BPM clock generator for Arduino
 *  @brief      A Library to implement BPM clock tick calls using hardware interruption. Supported and tested on AVR boards(ATmega168/328, ATmega16u4/32u4 and ATmega2560) and ARM boards(RPI2040, Teensy, Seedstudio XIAO M0 and ESP32)
 *  @version    2.1.0
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

// for extended steps in memory style and make use of 96ppqn for record propurse we can
// keep array[step] memory layout and add new information about note possition to be check for the entire ppqn pulse
// example: for a whole 24 pulses we only check array[step].offset that can vary from 0 to 24(ppqn/4)
// time/tick notation and representation notes:
// one quarter note == 4 steps in 16th notes step sequencer style
// PPQN / 4 = pulses in between steps(from step sequencer perspective, a quarter note have 4 steps)
// 24 PPQN (6 pulses per step)
// 48 PPQN (12 pulses per step)
// 96 PPQN (24 pulses per step)

// min: -(ppqn/4)-1 step, max: (ppqn/4)-1 steps  
// adjust the size of you template if more than 16 shuffle step info needed
#define MAX_SHUFFLE_TEMPLATE_SIZE   16
typedef struct {
    bool active = false;
    uint8_t size = MAX_SHUFFLE_TEMPLATE_SIZE;
    int8_t step[MAX_SHUFFLE_TEMPLATE_SIZE] = {0};
} SHUFFLE_TEMPLATE;

// for smooth slave tempo calculate display you should raise this value 
// in between 64 to 128.
// note: this doesn't impact on sync time, only display time getTempo()
// if you dont want to use it, set it to 1 for memory save
#define EXT_INTERVAL_BUFFER_SIZE 24

#define MIN_BPM	1
#define MAX_BPM	300

#define PHASE_FACTOR 16
#define PLL_X 220

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

class uClockClass {

    public:
        enum SyncMode {
            INTERNAL_CLOCK = 0,
            EXTERNAL_CLOCK
        };

        enum ClockState {
            PAUSED = 0,
            STARTING,
            STARTED
        };

        enum PPQNResolution {
            PPQN_24 = 24,
            PPQN_48 = 48,
            PPQN_96 = 96,
            PPQN_384 = 384,
            PPQN_480 = 480,
            PPQN_960 = 960
        };

        ClockState state;
        
        uClockClass();

        void setOnPPQN(void (*callback)(uint32_t tick)) {
            onPPQNCallback = callback;
        }

        void setOnStep(void (*callback)(uint32_t step)) {
            onStepCallback = callback;
        }
        
        void setOnSync24(void (*callback)(uint32_t tick)) {
            onSync24Callback = callback;
        }

        void setOnClockStart(void (*callback)()) {
            onClockStartCallback = callback;
        }

        void setOnClockStop(void (*callback)()) {
            onClockStopCallback = callback;
        }

        void init();
        void setPPQN(PPQNResolution resolution);

        void handleTimerInt();
        void handleExternalClock();
        void resetCounters();
        
        // external class control
        void start();
        void stop();
        void pause();
        void setTempo(float bpm);
        float getTempo();

        // external timming control
        void setMode(SyncMode tempo_mode);
        SyncMode getMode();
        void clockMe();

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

        // shuffle
        bool inline processShuffle();

        void (*onPPQNCallback)(uint32_t tick);
        void (*onStepCallback)(uint32_t step);
        void (*onSync24Callback)(uint32_t tick);
        void (*onClockStartCallback)();
        void (*onClockStopCallback)();

        // internal clock control
        // uint16_t ppqn;
        PPQNResolution ppqn = PPQN_96;
        uint32_t tick;
        uint32_t int_clock_tick;
        uint8_t mod24_counter;
        uint8_t mod24_ref;
        uint8_t mod_step_counter;
        uint8_t mod_step_ref;
        uint32_t step_counter; // should we go uint16_t?

        // external clock control
        volatile uint32_t ext_clock_us;
        volatile uint32_t ext_clock_tick;
        volatile uint32_t ext_interval;
        uint32_t last_interval;
        uint32_t sync_interval;

        float tempo;
        uint32_t start_timer;
        SyncMode mode;

        volatile uint32_t ext_interval_buffer[EXT_INTERVAL_BUFFER_SIZE];
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

