#include "Arduino.h"
#include <uClock.h>

// Sequencer config
#define STEP_MAX_SIZE      16
#define SEQUENCER_MIN_BPM  50
#define SEQUENCER_MAX_BPM  178
#define NOTE_VELOCITY      90
#define ACCENT_VELOCITY    110

// Choose only 1 mode and comment the other.
// OLD_SCHOOL_ACID_ACCENTED
// a button/led extra hardware to control steps velocity using accent only
// NEW_SCHOOL_VELOCITY_CONTROLLED
// a 10k potentiomer extra hardware to freely control steps velocity
#define OLD_SCHOOL_ACID_ACCENTED
#define NEW_SCHOOL_FREE_VELOCITY

typedef struct
{
  uint8_t note;
  uint8_t velocity;
  bool accent;
  bool glide;
  bool rest;
} SEQUENCER_STEP_DATA;

SEQUENCER_STEP_DATA _sequencer[STEP_MAX_SIZE];
uint16_t _last_step = 0;

// MIDI clock, start, stop, note on and note off byte definitions - based on MIDI 1.0 Standards.
#define MIDI_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP  0xFC
#define NOTE_ON    0x90
#define NOTE_OFF   0x80

void sendMidiMessage(uint8_t command, uint8_t byte1, uint8_t byte2)
{ 
  // send midi message
  Serial.write(command);
  Serial.write(byte1);
  Serial.write(byte2);
}

// The callback function wich will be called by uClock each Pulse of 16PPQN clock resolution.
// At this resolution each call represents exactly one step.
void ClockOut16PPQN(uint32_t * tick) 
{
  uint16_t step;
  
  // get actual step.
  step = *tick % STEP_MAX_SIZE;
  
  // send note off for the last step note on if we had send it on last ClockOut16PPQN() call and if this step are not in glide mode also.
  if ( _sequencer[_last_step].rest == false && _sequencer[_last_step].glide == false ) {
    sendMidiMessage(NOTE_OFF, _sequencer[_last_step].note, 0);
  }

  // send note on only if this step are not in rest mode
  if ( _sequencer[step].rest == false ) {
    sendMidiMessage(NOTE_ON, _sequencer[step].note, _sequencer[step].velocity);
  }

  // time to let glide go away? be shure to send glided note off before the _last_step send his note off
  // same note? do not send note off
  if ( _sequencer[_last_step].glide == true && _sequencer[step].note != _sequencer[_last_step].note ) {
    sendMidiMessage(NOTE_OFF, _sequencer[_last_step].note, 0);
  }

  _last_step = step;
}

// The callback function wich will be called by uClock each Pulse of 96PPQN clock resolution.
void ClockOut96PPQN(uint32_t * tick) 
{
  // Send MIDI_CLOCK to external gears
  Serial.write(MIDI_CLOCK);
}

// The callback function wich will be called when clock starts by using Clock.start() method.
void onClockStart() 
{
  Serial.write(MIDI_START);
}

// The callback function wich will be called when clock stops by using Clock.stop() method.
void onClockStop() 
{
  Serial.write(MIDI_STOP);
}

void setup() 
{
  // Initialize serial communication at 31250 bits per second, the default MIDI serial speed communication:
  Serial.begin(31250);

  // Inits the clock
  uClock.init();
  
  // Set the callback function for the clock output to send MIDI Sync message.
  uClock.setClock96PPQNOutput(ClockOut96PPQN);
  
  // Set the callback function for the step sequencer on 16ppqn
  uClock.setClock16PPQNOutput(ClockOut16PPQN);  
  
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStartOutput(onClockStart);  
  uClock.setOnClockStopOutput(onClockStop);
  
  // Set the clock BPM to 126 BPM
  uClock.setTempo(126);

  // initing sequencer data
  for ( uint16_t i = 0; i < STEP_MAX_SIZE; i++ ) {
    _sequencer[i].note = 36;
    _sequencer[i].velocity = NOTE_VELOCITY;
    _sequencer[i].accent = false;
    _sequencer[i].glide = false;
    _sequencer[i].rest = false;
  }

  // Starts the clock, tick-tac-tick-tac...
  uClock.start();
}

// User interaction goes here
void loop() 
{
  // octave pot
  // note pot
  // tempo pot
  // pattern step init pot
  // pattern step size pot
  // previous step button
  // next step button
  // rest button/led
  // glide button/led
  // accent button/led
}
