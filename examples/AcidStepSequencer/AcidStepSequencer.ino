// Acid StepSequencer, a Roland TB303 step sequencer engine clone
// author: midilab contact@midilab.co
// under MIT license
#include "Arduino.h"
#include <uClock.h>

// Sequencer config
#define STEP_MAX_SIZE      16
#define SEQUENCER_MIN_BPM  50
#define SEQUENCER_MAX_BPM  177
#define NOTE_LENGTH        4 // min: 1 max: 5 DO NOT EDIT BEYOND!!!
#define NOTE_VELOCITY      90
#define ACCENT_VELOCITY    127

// do not edit this!
#define NOTE_STACK_SIZE    3

// Ui config
#define LOCK_POT_SENSTIVITY 3

// MIDI modes
#define MIDI_CHANNEL      0 // 0 = channel 1
#define MIDI_MODE
//#define SERIAL_MODE

// hardware setup to fit different kinda of setups and arduino models
#define OCTAVE_POT_PIN            A3
#define NOTE_POT_PIN              A2
#define STEP_LENGTH_POT_PIN       A1
#define TEMPO_POT_PIN             A0

#define PREVIOUS_STEP_BUTTON_PIN  2
#define NEXT_STEP_BUTTON_PIN      3
#define REST_BUTTON_PIN           4
#define GLIDE_BUTTON_PIN          5
#define ACCENT_BUTTON_PIN         6
#define PLAY_STOP_BUTTON_PIN      7

#define PREVIOUS_STEP_LED_PIN     8
#define NEXT_STEP_LED_PIN         9
#define REST_LED_PIN              10
#define GLIDE_LED_PIN             11
#define ACCENT_LED_PIN            12
#define PLAY_STOP_LED_PIN         13

// Sequencer data
typedef struct
{
  uint8_t note;
  bool accent;
  bool glide;
  bool rest;
} SEQUENCER_STEP_DATA;

SEQUENCER_STEP_DATA _sequencer[STEP_MAX_SIZE];

typedef struct
{
  uint8_t note;
  int8_t length;
} STACK_NOTE_DATA;

STACK_NOTE_DATA _note_stack[NOTE_STACK_SIZE];

bool _playing = false;
uint16_t _step, _step_edit = 0;
uint16_t _step_length = STEP_MAX_SIZE;

// MIDI clock, start, stop, note on and note off byte definitions - based on MIDI 1.0 Standards.
#define MIDI_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP  0xFC
#define NOTE_ON    0x90
#define NOTE_OFF   0x80

// User Interface data
// 6 buttons to keep last value track
// 4 10k potentiometers to keep lasta value track
uint8_t _button_state[6] = {1};
uint16_t _pot_state[4] = {0};
bool _lock_pot[4] = {true};
uint8_t _last_octave = 3;
uint8_t _last_note = 0;
uint8_t _bpm_blink_timer = 1;

void sendMidiMessage(uint8_t command, uint8_t byte1, uint8_t byte2)
{ 
  // send midi message
  command = command | (uint8_t)MIDI_CHANNEL;
  Serial.write(command);
  Serial.write(byte1);
  Serial.write(byte2);
}

// The callback function wich will be called by uClock each Pulse of 16PPQN clock resolution.
// Each call represents exactly one step here.
void ClockOut16PPQN(uint32_t * tick) 
{
  uint16_t step, length;
  
  // get actual step.
  _step = *tick % _step_length;
  
  // send note on only if this step are not in rest mode
  if ( _sequencer[_step].rest == false ) {
    // send note on
    sendMidiMessage(NOTE_ON, _sequencer[_step].note, _sequencer[_step].accent ? ACCENT_VELOCITY : NOTE_VELOCITY);
    
    // check for glide event ahead of _step
    step = _step;
    for ( uint16_t i = 1; i < _step_length; i++  ) {
      ++step;
      step = step % _step_length;
      if ( _sequencer[step].glide == true && _sequencer[step].rest == false ) {
        length = NOTE_LENGTH + (i * 6);
        break;
      } else if ( _sequencer[step].rest == false ) {
        length = NOTE_LENGTH;
        break;
      }
    }

    // find a free note stack to fit in
    for ( uint8_t i = 0; i < NOTE_STACK_SIZE; i++ ) {
      if ( _note_stack[i].length == -1 ) {
        _note_stack[i].note = _sequencer[_step].note;
        _note_stack[i].length = length;
        return;
      }
    }

    // if we reach at this point means we could not find a free note stack for this note... so lets send note off to avoid ghost notes in the air
    sendMidiMessage(NOTE_OFF, _sequencer[_step].note, 0);
  }  
}

// The callback function wich will be called by uClock each Pulse of 96PPQN clock resolution.
void ClockOut96PPQN(uint32_t * tick) 
{
  // Send MIDI_CLOCK to external hardware
  Serial.write(MIDI_CLOCK);

  // handle note on stack
  for ( uint8_t i = 0; i < NOTE_STACK_SIZE; i++ ) {
    if ( _note_stack[i].length != -1 ) {
      --_note_stack[i].length;
      if ( _note_stack[i].length == 0 ) {
        sendMidiMessage(NOTE_OFF, _note_stack[i].note, 0);
        _note_stack[i].length = -1;
      }
    }  
  }

  // BPM led indicator
  if ( !(*tick % (96)) || (*tick == 0) ) {  // first compass step will flash longer
    _bpm_blink_timer = 8;
    digitalWrite(PLAY_STOP_LED_PIN , HIGH);
  } else if ( !(*tick % (24)) ) {   // each quarter led on
    digitalWrite(PLAY_STOP_LED_PIN , HIGH);
  } else if ( !(*tick % _bpm_blink_timer) ) { // get led off
    digitalWrite(PLAY_STOP_LED_PIN , LOW);
    _bpm_blink_timer = 1;
  }
}

// The callback function wich will be called when clock starts by using Clock.start() method.
void onClockStart() 
{
  Serial.write(MIDI_START);
  digitalWrite(PLAY_STOP_LED_PIN , LOW);
  _playing = true;
}

// The callback function wich will be called when clock stops by using Clock.stop() method.
void onClockStop() 
{
  Serial.write(MIDI_STOP);
  for ( uint8_t i = 0; i < NOTE_STACK_SIZE; i++ ) {
    sendMidiMessage(NOTE_OFF, _note_stack[i].note, 0);
    _note_stack[i].length = -1;
  }
  _playing = false;
}

void configureInterface()
{
  // Buttons config
  // use internal pullup for buttons
  pinMode(PREVIOUS_STEP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(NEXT_STEP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(REST_BUTTON_PIN, INPUT_PULLUP);
  pinMode(GLIDE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACCENT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PLAY_STOP_BUTTON_PIN, INPUT_PULLUP);

  // Leds config
  pinMode(PREVIOUS_STEP_LED_PIN, OUTPUT);
  pinMode(NEXT_STEP_LED_PIN, OUTPUT);
  pinMode(REST_LED_PIN, OUTPUT);
  pinMode(GLIDE_LED_PIN, OUTPUT);
  pinMode(ACCENT_LED_PIN, OUTPUT);
  pinMode(PLAY_STOP_LED_PIN, OUTPUT);

  digitalWrite(PREVIOUS_STEP_LED_PIN, LOW);
  digitalWrite(NEXT_STEP_LED_PIN, LOW);
  digitalWrite(REST_LED_PIN, LOW);
  digitalWrite(GLIDE_LED_PIN, LOW);
  digitalWrite(ACCENT_LED_PIN, LOW);
  digitalWrite(PLAY_STOP_LED_PIN, LOW);  
}

void setup() 
{
  // Initialize serial communication
#ifdef MIDI_MODE
  // the default MIDI serial speed communication at 31250 bits per second
  Serial.begin(31250); 
#endif
#ifdef SERIAL_MODE
  // for usage with a PC with a serial to MIDI bridge
  Serial.begin(115200);
#endif

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
    _sequencer[i].accent = false;
    _sequencer[i].glide = false;
    _sequencer[i].rest = false;
  }

  // initing note stack data
  for ( uint8_t i = 0; i < NOTE_STACK_SIZE; i++ ) {
    _note_stack[i].note = 0;
    _note_stack[i].length = -1;
  }

  // pins, buttons, leds and pots config
  configureInterface();

  acidRandomize();
}

void acidRandomize() 
{
  // ramdom it all
  for ( uint16_t i = 0; i < STEP_MAX_SIZE; i++ ) {
    _sequencer[i].note = random(36, 70); // octave 2 to 4. octave 3 to 5 (40 - 83)
    _sequencer[i].accent = random(0, 2);
    _sequencer[i].glide = random(0, 2);
    _sequencer[i].rest = random(0, 1);
  }
}

void sendPreviewNote(uint16_t step)
{
  unsigned long milliTime, preMilliTime;
  
  sendMidiMessage(NOTE_ON, _sequencer[step].note, _sequencer[step].accent ? ACCENT_VELOCITY : NOTE_VELOCITY);

  // avoid delay() call here because of uClock timmer1 usage
  //delay(200);
  preMilliTime = millis();
  while ( true ) {
    milliTime = millis();
    if (abs(milliTime - preMilliTime) >= 200) {
      break;
    }
  }
  
  sendMidiMessage(NOTE_OFF, _sequencer[step].note, 0);
}

void lockPotsState(bool state)
{
  for ( uint8_t i = 0; i < 4; i++ ) {
    _lock_pot[i] = state;
  }
}

bool pressed(uint8_t button_pin)
{
  uint8_t value;
  uint8_t * last_value;

  switch(button_pin) {
    case PREVIOUS_STEP_BUTTON_PIN:
      last_value = &_button_state[0];
      break;
    case NEXT_STEP_BUTTON_PIN:
      last_value = &_button_state[1];
      break;
    case REST_BUTTON_PIN:
      last_value = &_button_state[2];
      break;   
    case GLIDE_BUTTON_PIN:
      last_value = &_button_state[3];
      break;
    case ACCENT_BUTTON_PIN:
      last_value = &_button_state[4];
      break;
    case PLAY_STOP_BUTTON_PIN:
      last_value = &_button_state[5];
      break;    
    default:
      return false;                    
  }
  
  value = digitalRead(button_pin);
  
  // check, using pullup pressed button goes LOW
  if ( value != *last_value && value == LOW ) {
    *last_value = value; 
    return true;    
  } else {
    *last_value = value; 
    return false;
  }
   
}

int16_t getPotChanges(uint8_t pot_pin, uint16_t min_value, uint16_t max_value)
{
  uint16_t value;
  uint16_t * last_value;
  bool * lock_pot;
  uint8_t pot_sensitivity = 1;

  switch(pot_pin) {
    case OCTAVE_POT_PIN:
      last_value = &_pot_state[0];
      lock_pot = &_lock_pot[0];
      break;
    case NOTE_POT_PIN:
      last_value = &_pot_state[1];
      lock_pot = &_lock_pot[1];
      break;
    case STEP_LENGTH_POT_PIN:
      last_value = &_pot_state[2];
      lock_pot = &_lock_pot[2];
      break;   
    case TEMPO_POT_PIN:
      last_value = &_pot_state[3];
      lock_pot = &_lock_pot[3];
      break;
    default:
      return -1;
  }
    
  // range our value
  value = (analogRead(pot_pin) / (1024 / ((max_value - min_value) + 1))) + min_value;

  // a lock system to not mess with some data(pots are terrible for some kinda of user interface data controls)
  if ( *lock_pot == true ) {
      pot_sensitivity = LOCK_POT_SENSTIVITY;
  }
  
  if ( abs(value - *last_value) >= pot_sensitivity ) {
    *last_value = value; 
    if ( *lock_pot == true ) {
      *lock_pot = false;
    }
    return value;    
  } else {
    return -1;
  }  
}

void processPots()
{
  int8_t octave, note, step_note;
  int16_t tempo, step_length;

  octave = getPotChanges(OCTAVE_POT_PIN, 0, 10);
  if ( octave != -1 ) {  
    _last_octave = octave;
  }

  note = getPotChanges(NOTE_POT_PIN, 0, 11);
  if ( note != -1 ) { 
    _last_note = note;
  }

  // changes on octave or note pot?
  if ( octave != -1 || note != -1 ) {
    _sequencer[_step_edit].note = (_last_octave * 8) + _last_note;
    if ( _playing == false && _sequencer[_step_edit].rest == false ) {
      sendPreviewNote(_step_edit);
    }
  }

  step_length = getPotChanges(STEP_LENGTH_POT_PIN, 1, STEP_MAX_SIZE);
  if ( step_length != -1 ) {  
    _step_length = step_length;
    if ( _step_edit >= _step_length ) {
      _step_edit = _step_length-1;
    }
  }

  tempo = getPotChanges(TEMPO_POT_PIN, SEQUENCER_MIN_BPM, SEQUENCER_MAX_BPM);
  if ( tempo != -1 ) {   
    uClock.setTempo(tempo);
  }
}
  
void processButtons()
{
  // play/stop
  if ( pressed(PLAY_STOP_BUTTON_PIN) ) {
    if ( _playing == false ) {
      // Starts the clock, tick-tac-tick-tac...
      uClock.start();
    } else {
      // stop the clock
      uClock.stop();
    }
  }

  // previous step edit
  if ( pressed(PREVIOUS_STEP_BUTTON_PIN) ) {
    if ( _step_edit != 0 ) {
      // add a lock here for octave and note to not mess with edit mode when moving steps around 
      lockPotsState(true);   
      --_step_edit;
    }
    if ( _playing == false && _sequencer[_step_edit].rest == false ) {
      sendPreviewNote(_step_edit);
    }
  }

  // next step edit
  if ( pressed(NEXT_STEP_BUTTON_PIN) ) {
    if ( _step_edit < _step_length-1 ) {
      // add a lock here for octave and note to not mess with edit mode when moving steps around
      lockPotsState(true);     
      ++_step_edit;
    }
    if ( _playing == false && _sequencer[_step_edit].rest == false ) {
      sendPreviewNote(_step_edit);
    }    
  }

  // step rest
  if ( pressed(REST_BUTTON_PIN) ) {
    _sequencer[_step_edit].rest = !_sequencer[_step_edit].rest;
    if ( _playing == false && _sequencer[_step_edit].rest == false ) {
      sendPreviewNote(_step_edit);
    }
  }

  // step glide
  if ( pressed(GLIDE_BUTTON_PIN) ) {
    _sequencer[_step_edit].glide = !_sequencer[_step_edit].glide;
  }

  // step accent
  if ( pressed(ACCENT_BUTTON_PIN) ) {
    _sequencer[_step_edit].accent = !_sequencer[_step_edit].accent;
    if ( _playing == false && _sequencer[_step_edit].rest == false ) {
      sendPreviewNote(_step_edit);
    }       
  }     
}
  
void processLeds()
{   
  // Editing First Step? 
  if ( _step_edit == 0 ) {
    digitalWrite(PREVIOUS_STEP_LED_PIN , HIGH);
  } else {
    digitalWrite(PREVIOUS_STEP_LED_PIN , LOW);
  }  

  // Editing Last Step? 
  if ( _step_edit == _step_length-1 ) {
    digitalWrite(NEXT_STEP_LED_PIN , HIGH);
  } else {
    digitalWrite(NEXT_STEP_LED_PIN , LOW);
  }  
  
  // Rest 
  if ( _sequencer[_step_edit].rest == true ) {
    digitalWrite(REST_LED_PIN , HIGH);
  } else {
    digitalWrite(REST_LED_PIN , LOW);
  }

  // Glide 
  if ( _sequencer[_step_edit].glide == true ) {
    digitalWrite(GLIDE_LED_PIN , HIGH);
  } else {
    digitalWrite(GLIDE_LED_PIN , LOW);
  }  

  // Accent 
  if ( _sequencer[_step_edit].accent == true ) {
    digitalWrite(ACCENT_LED_PIN , HIGH);
  } else {
    digitalWrite(ACCENT_LED_PIN , LOW);
  } 

  // shut down play led if we are stoped
  if ( _playing == false ) {
    digitalWrite(PLAY_STOP_LED_PIN , LOW);
  }
}

// User interaction goes here
void loop() 
{
  processButtons();
  processLeds();
  processPots();
}
