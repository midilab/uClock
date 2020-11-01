/*!
 *  @file       uClock.cpp
 *  Project     BPM clock generator for Arduino
 *  @brief      A Library to implement BPM clock tick calls using hardware timer1 interruption. Tested on ATmega168/328, ATmega16u4/32u4 and ATmega2560.
 *              Derived work from mididuino MidiClock class. (c) 2008 - 2011 - Manuel Odendahl - wesen@ruinwesen.com
 *  @version    0.9.4
 *  @author     Romulo Silva
 *  @date       08/21/2020
 *  @license    MIT - (c) 2020 - Romulo Silva - contact@midilab.co
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

#define ATOMIC(X) noInterrupts(); X; interrupts();

// 
// Timer setup
// Work clock at: 62.5kHz/16usec
//
#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
IntervalTimer _teensyTimer;
void teensyInterrupt();
void initTeensyTimer()
{
	// 62500Hz
	_teensyTimer.begin(teensyInterrupt, 16);
	// Set the interrupt priority level, controlling which other interrupts
	// this timer is allowed to interrupt. Lower numbers are higher priority, 
	// with 0 the highest and 255 the lowest. Most other interrupts default to 128. 
	// As a general guideline, interrupt routines that run longer should be given 
	// lower priority (higher numerical values).
	_teensyTimer.priority(0);
}
#else
void initArduinoTimer()
{
	//
	// Configure timers and prescale
	// Timmer1: ATMega128, ATMega328, AtMega16U4 and AtMega32U4
	// Clock Speed Selection
	// CS10: Clock (No prescaling)	
	// Waveform Generation Mode (WGM) 16-bit timer settings
	// (WGM10, WGM12) Mode 5
	// Fast Pulse Width Modulation (PWM), 8-bit: 
	// TOP: 0x00FF (255)
	// OCR1x Update: BOTTOM
	// TOV1 Flag: TOP 
	// Overflow Interrupt Enable
	ATOMIC(
		TCCR1A = 0;
		TCCR1A = _BV(WGM10);
		TCCR1B = 0;
		TCCR1B = _BV(CS10) | _BV(WGM12);
		TIMSK1 |= _BV(TOIE1);
	)
}
#endif

void initWorkTimer() {
#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
	initTeensyTimer();
#else
	initArduinoTimer();
#endif
}

namespace umodular { namespace clock {

static inline uint32_t phase_mult(uint32_t val) 
{
	return (val * PHASE_FACTOR) >> 8;
}

static inline uint16_t clock_diff(uint16_t old_clock, uint16_t new_clock) 
{
	if (new_clock >= old_clock) {
		return new_clock - old_clock;
	} else {
		return new_clock + (65535 - old_clock);
	}
}

uClockClass::uClockClass()
{
	// 11 is good for native 31250bps midi interface
	// 4 is good for usb-to-midi hid on leonardo
	// 1 is good on teensy lc usb midi
	internal_drift = 11;
	external_drift = 11;
	tempo = 120;
	pll_x = 220;
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

	// first interval calculus
	setTempo(tempo);
}

void uClockClass::init() 
{
	initWorkTimer();
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
	
	if (tempo == bpm) {
		return;
	}
	
	if (bpm > 300 || bpm < 10) {
		return;
	}

	tempo = bpm;

	ATOMIC(
		interval = (uint16_t)((156250.0 / tempo) - internal_drift);
		//interval = 62500 / (tempo * 24 / 60) - internal_drift;
	)
}

float uClockClass::getTempo() 
{
	if (mode == EXTERNAL_CLOCK) {
		uint32_t acc = 0;
		uint8_t acc_counter = 0;
		for (uint8_t i=0; i < EXT_INTERVAL_BUFFER_SIZE; i++) {
			if ( ext_interval_buffer[i] != 0) {
				acc += ext_interval_buffer[i];
				++acc_counter;
			}
		}
		if (acc != 0) {
			// get average interval, because MIDI sync world is a wild place...
			tempo = (float)(156250.0 / ((acc / acc_counter) + external_drift));
		}
	}
	return tempo;
}

void uClockClass::setDrift(uint8_t internal, uint8_t external)
{
	ATOMIC(
		internal_drift = internal;
		external_drift = external == 255 ? internal : external;
	)
}

uint8_t uClockClass::getMode() 
{
	return mode;
}

void uClockClass::setMode(uint8_t tempo_mode) 
{
	mode = tempo_mode;
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
	counter = 0;
	last_clock = 0;
	div96th_counter = 0;
	div32th_counter = 0;
	div16th_counter = 0;
	mod6_counter = 0;
	indiv96th_counter = 0;
	inmod6_counter = 0;
	ext_interval_buffer[EXT_INTERVAL_BUFFER_SIZE] = {0};
	ext_interval_idx = 0;
}

// TODO: Tap stuff
void uClockClass::tap() 
{
	// tap me
}

// TODO: Shuffle stuff
void uClockClass::shuffle() 
{
	// shuffle me
}

void uClockClass::handleExternalClock() 
{
	last_interval = clock_diff(last_clock, _clock);
	last_clock = _clock;

	indiv96th_counter++;
	inmod6_counter++;

	if (inmod6_counter == 6) {
		inmod6_counter = 0;
	}

	switch (state) {

		case PAUSED:
			break;

		case STARTING:
			state = STARTED;
			break;

		case STARTED:
			if (indiv96th_counter == 2) {
				interval = last_interval;
			} else {
				interval = (((uint32_t)interval * (uint32_t)pll_x) + (uint32_t)(256 - pll_x) * (uint32_t)last_interval) >> 8;
			}
			// accumulate interval incomming ticks data(for a better getTempo stability over bad clocks)
			ext_interval_buffer[ext_interval_idx] = interval;
			ext_interval_idx = ++ext_interval_idx % EXT_INTERVAL_BUFFER_SIZE;
			break;
	}
}

void uClockClass::handleTimerInt()  
{
	if (counter == 0) {
		
		counter = interval;

		if (onClock96PPQNCallback) {
			onClock96PPQNCallback(&div96th_counter);
		}
		
		if (mod6_counter == 0) {
			if (onClock32PPQNCallback) {
				onClock32PPQNCallback(&div32th_counter);
			}			
			if (onClock16PPQNCallback) {
				onClock16PPQNCallback(&div16th_counter);
			}
			div16th_counter++;
			div32th_counter++;
		}

		if (mod6_counter == 3) {
			if (onClock32PPQNCallback) {
				onClock32PPQNCallback(&div32th_counter);
			}
			div32th_counter++;
		}
		
		div96th_counter++;
		mod6_counter++;
	
		if (mode == EXTERNAL_CLOCK) {
			sync_interval = clock_diff(last_clock, _clock);
			if ((div96th_counter < indiv96th_counter) || (div96th_counter > (indiv96th_counter + 1))) {
				div96th_counter = indiv96th_counter;
				mod6_counter = inmod6_counter;
			}
			if (div96th_counter <= indiv96th_counter) {
				counter -= phase_mult(sync_interval);
			} else {
				if (counter > sync_interval) {
					counter += phase_mult(counter - sync_interval);
				}
			}
		}
		
		if (mod6_counter == 6) {
			mod6_counter = 0;
		}
		
	} else {
		counter--;
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

volatile uint16_t _clock = 0;
volatile uint32_t _timer = 0;

//
// TIMER INTERRUPT HANDLER 
// Clocked at: 62.5kHz/16usec
//
#if defined(TEENSYDUINO) && !defined(__AVR_ATmega32U4__)
void teensyInterrupt() 
#else
ISR(TIMER1_OVF_vect) 
#endif
{
	// global timer counter
	_timer = millis();
	
	if (uClock.state == umodular::clock::STARTED) {
		_clock++;
		uClock.handleTimerInt();
	}
}
