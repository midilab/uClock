# uClock

The **uClock BPM Generator library** is designed to implement precise and reliable BPM clock tick calls using the microcontroller's timer hardware interruption. It is designed to be multi-architecture, portable, and easy to use within the open source community universe. 

We have chosen PlatformIO and Arduino as our official deployment platforms. The library has been supported and tested on general **AVR boards (ATmega168/328, ATmega16u4/32u4, and ATmega2560)** as well as **ARM boards (Teensy, STM32XX, ESP32, Raspberry Pico, Seedstudio XIAO M0 and RP2040)**. 

The absence of real-time features necessary for creating professional-level embedded devices for music and video on open source community-based platforms like Arduino led to the development of uClock. By leveraging the use of timer hardware interruptions, the library can schedule and manage real-time-like processing with safe shared resource access through its API.

With uClock, you gain the ability to create professional-grade sequencers, sync boxes, or generate a precise BPM clock for external devices in the realms of music, audio/video productions, performances, or tech art installations. The library offers an external synchronization schema that enables you to generate an internal clock based on an external clock source, allowing you to master your entire MIDI setup or any other protocols according to your specific preferences and requirements.

## Interface
The uClock library API operates through attached callback functions mechanism:

1. **setOnPPQN(onPPQNCallback) > onPPQNCallback(uint32_t tick)** calls on each new pulse based on selected PPQN resolution (if no PPQN set, the default is 96PPQN)
2. **setOnStep(onStepCallback) > onStepCallback(uint32_t step)** good way to code old style step sequencer based on 16th note schema (not dependent on PPQN resolution)
3. **setOnSync24(onSync24Callback) > onSync24Callback(uint32_t tick)** good way to code a clock machine, or keep your devices synced with your device
4. **setOnClockStart(onClockStartCallback) > onClockStartCallback()** on uClock Start event
5. **setOnClockStop(onClockStopCallback) > onClockStopCallback()** on uClock Stop event

### Software Timer mode - for unsupported boards (or avoiding usage of interrupts)
If a supported board isn't detected during compilation then a generic fallback approach will be used. This does not utilise any interrupts and so does not ensure accurate timekeeping.  This can be useful to port your projects to boards that do not have support in uClock yet, or to test if suspected bugs in your code are related to interactions with interrupts or task handling.

You can force this non-interrupt "software timer mode" even on supported boards by defining the build flag `USE_UCLOCK_SOFTWARE_TIMER`.

In order for software timer mode to work, you need to add a call to your `loop()` function to process ticks. For example,

```c++
void loop() {
  uClock.run();
  
  // do anything else you need to do inside loop()...
  // you can intercalate your main processing with other uClock.run() calls to avoid timming accuracy loss.
  //uClock.run();
  // do anything other inside loop()...
  //uClock.run();
  // the faster you can call uClock.run() without blocking the better and accurate timming you can achieve.
}
```

## Set your own resolution for your clock needs

1. **PPQN_24** 24 Pulses Per Quarter Note
2. **PPQN_48** 48 Pulses Per Quarter Note
3. **PPQN_96** 96 Pulses Per Quarter Note
1. **PPQN_384** 384 Pulses Per Quarter Note
2. **PPQN_480** 480 Pulses Per Quarter Note
3. **PPQN_960** 960 Pulses Per Quarter Note

To generate a MIDI sync signal and synchronize external MIDI devices, you can start working with the resolution of 24PPQN, which aligns with the clocking standards of modern MIDI-syncable devices commonly available in the market. By sending 24 pulses per quarter note interval, you can ensure effective synchronization among your MIDI devices.

If you are working on the development of a vintage-style step sequencer, utilizing a resolution of 96PPQN is a fitting option to initiate the coding process. Then you can use onStepCallback call which corresponds to a step played, note or event.

Furthermore, it is possible to utilize all three resolutions simultaneously, allowing for flexibility based on your specific requirements and preferences.

## uClock v2.0 Breaking Changes

If you are coming from uClock version < 2.0 versions, pay attention to the breaking changes so you can update your code to reflect the new API interface:

### setCallback function name changes

- **setClock96PPQNOutput(onClock96PPQNOutputCallback)** is now _setOnPPQN(onPPQNCallback)_ and this clock depends on the PPQN setup using _setPPQN(clockPPQNResolution)_. For clock setup you now use a separated callback via _setOnSync24(onSync24Callback)_
- **setClock16PPQNOutput(ClockOut16PPQN)** is now _setOnStep(onStepCall)_ and it's not dependent on clock PPQN resolution  
- **setOnClockStartOutput(onClockStartCallback)** is now _setOnClockStart(onClockStartCallback)_
- **setOnClockStopOutput(onClockStopCallback)** is now _setOnClockStop(onClockStopCallback)_

### Tick resolution and sequencers

If created a device using setClock16PPQNOutput only you just change the API call to setOnStep. If you were dependent on setClock96PPQNOutput you might need to review your tick counting system, depending on which PPQN clock resolution you choose to use.

# Examples

You will find more complete examples on examples/ folder:  

```c++
#include <uClock.h>

// external or internal sync?
bool _external_sync_on = false;

// the main uClock PPQN resolution ticking
void onPPQNCallback(uint32_t tick) {
  // tick your sequencers or tickable devices...
}

void onStepCallback(uint32_t step) {
  // triger step data for sequencer device...
}

// The callback function called by uClock each Pulse of 24PPQN clock resolution.
void onSync24Callback(uint32_t tick) {
  // send sync signal to...
}

// The callback function called when clock starts by using uClock.start() method.
void onClockStartCallback() {
  // send start signal to...
}

// The callback function called when clock stops by using uClock.stop() method.
void onClockStopCallback() {
  // send stop signal to...
}

void setup() {

  // inits the clock library
  uClock.init();

  // avaliable resolutions
  // [ uClock.PPQN_24, uClock.PPQN_48, uClock.PPQN_96, uClock.PPQN_384, uClock.PPQN_480, uClock.PPQN_960 ]
  // not mandatory to call, the default is 96PPQN if not set
  uClock.setPPQN(uClock.PPQN_96);

  // you need to use at least one!
  uClock.setOnPPQN(onPPQNCallback);
  uClock.setOnStep(onStepCallback);
  uClock.setOnSync24(onSync24Callback);

  uClock.setOnClockStart(onClockStartCallback);
  uClock.setOnClockStop(onClockStopCallback);

  // set external sync mode?
  if (_external_sync_on) {
    uClock.setMode(uClock.EXTERNAL_CLOCK);
  }

  // starts clock
  uClock.start();
}

void loop() {
  // do we need to external sync?
  if (_external_sync_on) {
    // watch for external sync signal income
    bool signal_income = true; // your external input signal check will be this condition result
    if (signal_income) {
      // at each clockMe call uClock will process and handle external/internal syncronization
      uClock.clockMe();
    }
  }
}
```

## MIDI Examples

Here a few examples on the usage of Clock library for MIDI devices, keep in mind the need to make your own MIDI interface, more details will be avaliable soon but until that, you can find good material over the net about the subject.

If you don't have native USB/MIDI support on your microcontroller and don't want to build a MIDI interface and you are going to use your arduino only with your PC, you can use a Serial-to-Midi bridge and connects your arduino via USB cable to your conputer to use it as a MIDI tool [like this one](http://projectgus.github.io/hairless-midiserial/).

### A Simple MIDI Sync Box sketch example

Here is an example on how to create a simple MIDI Sync Box on Arduino boards

```c++
#include <uClock.h>

// MIDI clock, start and stop byte definitions - based on MIDI 1.0 Standards.
#define MIDI_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP  0xFC

// The callback function called by Clock each Pulse of 24PPQN clock resolution.
void onSync24Callback(uint32_t tick) {
  // Send MIDI_CLOCK to external gears
  Serial.write(MIDI_CLOCK);
}

// The callback function called when clock starts by using Clock.start() method.
void onClockStart() {
  Serial.write(MIDI_START);
}

// The callback function called when clock stops by using Clock.stop() method.
void onClockStop() {
  Serial.write(MIDI_STOP);
}

void setup() {

  // Initialize serial communication at 31250 bits per second, the default MIDI serial speed communication:
  Serial.begin(31250);

  // Inits the clock
  uClock.init();
  // Set the callback function for the clock output to send MIDI Sync message based on 24PPQN
  uClock.setOnSync24(onSync24Callback);
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStartOutput(onClockStart);  
  uClock.setOnClockStopOutput(onClockStop);
  // Set the clock BPM to 126 BPM
  uClock.setTempo(126);

  // Starts the clock, tick-tac-tick-tac...
  uClock.start();

}

// Do it whatever to interface with Clock.stop(), Clock.start(), Clock.setTempo() and integrate your environment...
void loop() {

}
```

An example on how to create a simple MIDI Sync Box on Teensy boards and USB Midi setup. Select "MIDI" from the Tools->USB Type menu for Teensy to becomes a USB MIDI first.

```c++
#include <uClock.h>

// The callback function called by Clock each Pulse of 96PPQN clock resolution.
void onSync24Callback(uint32_t tick) {
  // Send MIDI_CLOCK to external gears
  usbMIDI.sendRealTime(usbMIDI.Clock);
}

// The callback function called when clock starts by using Clock.start() method.
void onClockStart() {
  usbMIDI.sendRealTime(usbMIDI.Start);
}

// The callback function called when clock stops by using Clock.stop() method.
void onClockStop() {
  usbMIDI.sendRealTime(usbMIDI.Stop);
}

void setup() {
  // Inits the clock
  uClock.init();
  // Set the callback function for the clock output to send MIDI Sync message. based on 24PPQN
  uClock.setOnSync24(onSync24Callback);
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStartOutput(onClockStart);  
  uClock.setOnClockStopOutput(onClockStop);
  // Set the clock BPM to 126 BPM
  uClock.setTempo(126);
  // Starts the clock, tick-tac-tick-tac...
  uClock.start();
}

// Do it whatever to interface with Clock.stop(), Clock.start(), Clock.setTempo() and integrate your environment...
void loop() {

}
```

### Acid Step Sequencer

A clone of Roland TB303 step sequencer main engine, here is an example with no user interface for interaction. If you're looking for a user interactable TB303 sequencer engine clone with user interface please take a look here https://github.com/midilab/uClock/tree/master/examples/AcidStepSequencer.

```c++
// Roland TB303 Step Sequencer engine clone.
// No interface here, just the engine as example.
// author: midilab contact@midilab.co
// Under MIT license
#include "Arduino.h"
#include <uClock.h>

// Sequencer config
#define STEP_MAX_SIZE      16
#define NOTE_LENGTH        12 // min: 1 max: 23 DO NOT EDIT BEYOND!!! 12 = 50% on 96ppqn, same as original tb303. 62.5% for triplets time signature
#define NOTE_VELOCITY      90
#define ACCENT_VELOCITY    127

// MIDI config
#define MIDI_CHANNEL      0 // 0 = channel 1

// do not edit below!
#define NOTE_STACK_SIZE    3

// MIDI clock, start, stop, note on and note off byte definitions - based on MIDI 1.0 Standards.
#define MIDI_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP  0xFC
#define NOTE_ON    0x90
#define NOTE_OFF   0x80

// Sequencer data
typedef struct
{
  uint8_t note;
  bool accent;
  bool glide;
  bool rest;
} SEQUENCER_STEP_DATA;

typedef struct
{
  uint8_t note;
  int8_t length;
} STACK_NOTE_DATA;

// main sequencer data
SEQUENCER_STEP_DATA _sequencer[STEP_MAX_SIZE];
STACK_NOTE_DATA _note_stack[NOTE_STACK_SIZE];
uint16_t _step_length = STEP_MAX_SIZE;

// make sure all above sequencer data are modified atomicly only
// eg. ATOMIC(_sequencer[0].accent = true); ATOMIC(_step_length = 7);
#define ATOMIC(X) noInterrupts(); X; interrupts();

// shared data to be used for user interface feedback
bool _playing = false;
uint16_t _step = 0;

void sendMidiMessage(uint8_t command, uint8_t byte1, uint8_t byte2)
{ 
  // send midi message
  command = command | (uint8_t)MIDI_CHANNEL;
  Serial.write(command);
  Serial.write(byte1);
  Serial.write(byte2);
}

// The callback function called by uClock each Pulse of 16PPQN clock resolution. Each call represents exactly one step.
void onStepCallback(uint32_t tick) 
{
  uint16_t step;
  uint16_t length = NOTE_LENGTH;
  
  // get actual step.
  _step = tick % _step_length;
  
  // send note on only if this step are not in rest mode
  if ( _sequencer[_step].rest == false ) {

    // check for glide event ahead of _step
    step = _step;
    for ( uint16_t i = 1; i < _step_length; i++  ) {
      ++step;
      step = step % _step_length;
      if ( _sequencer[step].glide == true && _sequencer[step].rest == false ) {
        length = NOTE_LENGTH + (i * 24);
        break;
      } else if ( _sequencer[step].rest == false ) {
        break;
      }
    }

    // find a free note stack to fit in
    for ( uint8_t i = 0; i < NOTE_STACK_SIZE; i++ ) {
      if ( _note_stack[i].length == -1 ) {
        _note_stack[i].note = _sequencer[_step].note;
        _note_stack[i].length = length;
        // send note on
        sendMidiMessage(NOTE_ON, _sequencer[_step].note, _sequencer[_step].accent ? ACCENT_VELOCITY : NOTE_VELOCITY);    
        return;
      }
    }
  }  
}

// The callback function called by uClock each Pulse of 96PPQN clock resolution.
void onPPQNCallback(uint32_t tick) 
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
}

// The callback function called when clock starts by using Clock.start() method.
void onClockStart() 
{
  Serial.write(MIDI_START);
  _playing = true;
}

// The callback function called when clock stops by using Clock.stop() method.
void onClockStop() 
{
  Serial.write(MIDI_STOP);
  // send all note off on sequencer stop
  for ( uint8_t i = 0; i < NOTE_STACK_SIZE; i++ ) {
    sendMidiMessage(NOTE_OFF, _note_stack[i].note, 0);
    _note_stack[i].length = -1;
  }
  _playing = false;
}

void setup() 
{
  // Initialize serial communication
  // the default MIDI serial speed communication at 31250 bits per second
  Serial.begin(31250); 

  // Inits the clock
  uClock.init();
  
  // Set the callback function for the clock output to send MIDI Sync message.
  uClock.setOnPPQN(onPPQNCallback);
  
  // Set the callback function for the step sequencer on 16ppqn
  uClock.setOnStep(onStepCallback);  
  
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStart(onClockStart);  
  uClock.setOnClockStop(onClockStop);
  
  // Set the clock BPM to 126 BPM
  uClock.setTempo(126);

  // initing sequencer data
  for ( uint16_t i = 0; i < STEP_MAX_SIZE; i++ ) {
    _sequencer[i].note = 48;
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
  //configureYourUserInterface();
  
  // start sequencer
  uClock.start();
}

// User interaction goes here
void loop() 
{
  // Controls your 303 engine interacting with user here...
  // you can change data by using _sequencer[] and _step_length only! do not mess with _note_stack[]!
  // IMPORTANT!!! Sequencer main data are used inside a interrupt enabled by uClock for BPM clock timing. Make sure all sequencer data are modified atomicly using this macro ATOMIC();
  // eg. ATOMIC(_sequencer[0].accent = true); ATOMIC(_step_length = 7);
  //processYourButtons();
  //processYourLeds();
  //processYourPots();
}
```
