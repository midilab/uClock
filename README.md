# uClock

**BPM clock generator for Arduino** is a library to implement BPM clock tick calls using **hardware timer1 interruption** for tight and solid timming clock ticks. Tested on ATmega168/328, ATmega16u4/32u4 and ATmega2560.

Generate your self tight BPM clock for music, audio/video productions, performances or instalations. You can clock your MIDI setup or sync different protocols as you wish.

## Interface
Clock library interfaces via attached callback function running on a hardware interrupt and is able to process the following resolutions:

1. **16PPQN** 16 Pulses Per Quarter Note
2. **32PPQN** 32 Pulses Per Quarter Note
3. **96PPQN** 96 Pulses Per Quarter Note

To generate a MIDI sync signal to sync external MIDI devices for example, you need to work with the resolution of 96PPQN to follow the standards of MIDI protocol that handles the clock based on 24PPQN.

For a simple old felling step sequencer a 16PPQN resolution is a good way to start coding your own step sequencer.

You can also use all the 3 resolutions at the same time for whatever reason you think you should.

## Examples
Here a few examples on the usage of Clock library for MIDI devices, keep in mind the need to make your own MIDI interface, more details will be avaliable soon but until that, you can find good material over the net about the subject.

If you dont want to build a MIDI interface and you are going to use your arduino only with your PC, you can use a Serial-to-Midi bridge and connects your arduino via USB cable to your conputer to use it as a MIDI tool [like this one](http://projectgus.github.io/hairless-midiserial/).

### A Simple MIDI Sync Box sketch example
Here is a example on how to create a simple MIDI Sync Box

```c++
#include <uClock.h>

// MIDI clock, start and stop byte definitions - based on MIDI 1.0 Standards.
#define MIDI_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP  0xFC

// The callback function wich will be called by Clock each Pulse of 96PPQN clock resolution.
void ClockOut96PPQN(uint32_t * tick) {
  // Send MIDI_CLOCK to external gears
  Serial.write(MIDI_CLOCK);
}

// The callback function wich will be called when clock starts by using Clock.start() method.
void onClockStart() {
  Serial.write(MIDI_START);
}

// The callback function wich will be called when clock stops by using Clock.stop() method.
void onClockStop() {
  Serial.write(MIDI_STOP);
}

void setup() {

  // Initialize serial communication at 31250 bits per second, the default MIDI serial speed communication:
  Serial.begin(31250);

  // Inits the clock
  uClock.init();
  // Set the callback function for the clock output to send MIDI Sync message.
  uClock.setClock96PPQNOutput(ClockOut96PPQN);
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

### A Simple Step Sequencer
A simple step sequencer for MIDI devices:


```c++
Soon...
```

