/*!
 *  @file       uClock.h
 *  Project     BPM clock generator for Arduino
 *  @brief      A Library to implement BPM clock tick calls using hardware interruption. Supported and tested on AVR boards(ATmega168/328, ATmega16u4/32u4 and ATmega2560) and ARM boards(Teensy, Seedstudio XIAO M0 and ESP32)
 *  @version    1.4.2
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

#ifndef __U_CLOCK_H__
#define __U_CLOCK_H__

#include <Arduino.h>
#include <inttypes.h>

namespace umodular { namespace clock {

// min: 2 step, max: 16 steps  
// adjust the size of you template if more than 16 needed
// step adjust goes min: -5, max: 5
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
private:
    
    float inline freqToBpm(uint32_t freq);

    // shuffle
    uint8_t inline processShuffle();

    void (*onClock96PPQNCallback)(uint32_t tick);
    void (*onClock32PPQNCallback)(uint32_t tick);
    void (*onClock16PPQNCallback)(uint32_t tick);
    void (*onClockStartCallback)();
    void (*onClockStopCallback)();

    // internal clock control
    uint32_t internal_tick;
    uint32_t div32th_counter;
    uint32_t div16th_counter;
    uint8_t mod6_counter;

    // external clock control
    volatile uint32_t external_clock;
    volatile uint32_t external_tick;
    volatile uint32_t indiv32th_counter;
    volatile uint32_t indiv16th_counter;
    volatile uint8_t inmod6_counter;
    volatile uint32_t interval;
    uint32_t last_interval;
    uint32_t sync_interval;

    float tempo;
    uint32_t start_timer;
    uint8_t mode;

    volatile uint32_t ext_interval_buffer[EXT_INTERVAL_BUFFER_SIZE];
    uint16_t ext_interval_idx;

    // shuffle implementation that applies to 16PPQN callback
    volatile SHUFFLE_TEMPLATE shuffle;
    bool shuffle_shoot_ctrl = true;
    volatile int8_t shuffle_length_ctrl = 0;

public:

    enum {
        INTERNAL_CLOCK = 0,
        EXTERNAL_CLOCK
    };

    enum {
        PAUSED = 0,
        STARTING,
        STARTED
    };

    uint8_t state;
    
    uClockClass();

    void setClock96PPQNOutput(void (*callback)(uint32_t tick)) {
        onClock96PPQNCallback = callback;
    }
    
    void setClock32PPQNOutput(void (*callback)(uint32_t tick)) {
        onClock32PPQNCallback = callback;
    }

    void setClock16PPQNOutput(void (*callback)(uint32_t tick)) {
        onClock16PPQNCallback = callback;
    }

    void setOnClockStartOutput(void (*callback)()) {
        onClockStartCallback = callback;
    }

    void setOnClockStopOutput(void (*callback)()) {
        onClockStopCallback = callback;
    }

    void init();
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
    void setMode(uint8_t tempo_mode);
    uint8_t getMode();
    void clockMe();

    // shuffle
    void setShuffle(bool active);
    bool isShuffled();
    void setShuffleSize(uint8_t size);
    void setShuffleData(uint8_t step, int8_t tick);
    void setShuffleTemplate(int8_t * shuff);
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
};

} } // end namespace umodular::clock

extern umodular::clock::uClockClass uClock;

extern "C" {
    extern volatile uint16_t _clock;
    extern volatile uint32_t _timer;
}

#endif /* __U_CLOCK_H__ */

