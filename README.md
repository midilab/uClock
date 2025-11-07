# uClock - BPM Clock Generator Library

A professional-grade BPM clock generator library for Arduino and compatible microcontrollers, designed for musicians, artists, and engineers creating sequencers, sync boxes, and real-time musical devices.

## Overview

**uClock** delivers precise, hardware-interrupt-driven clock timing for music and media applications. Whether you're building a MIDI sequencer, modular synthesizer clock, or synchronizing multiple devices, uClock provides the rock-solid timing foundation your project needs.

The library leverages hardware timer interrupts to ensure accurate BPM generation and synchronization, making it suitable for professional music production, live performance, and creative installations.

## Supported Platforms

- **AVR**: ATmega168/328, ATmega16u4/32u4, ATmega2560
- **ARM**: Teensy (all versions), STM32XX, Seeed Studio XIAO M0
- **ESP32**: All ESP32 family boards
- **RP2040**: Raspberry Pi Pico and compatible boards

## Why uClock?

Open-source platforms like Arduino and PlatformIO traditionally lack the real-time capabilities required for professional music and video applications. uClock bridges this gap by providing:

- **Precise timing** through hardware interrupts
- **Flexible clock resolutions** from 1 to 960 PPQN
- **External sync support** for master/slave configurations
- **Shuffle and groove** capabilities for humanized timing
- **Multi-track sequencing** with independent shuffle per track
- **Multiple sync outputs** for different device standards

## Installation

### PlatformIO
1. Open platformio.ini, a project configuration file located in the root of PlatformIO project.
2. Add the following line to the lib_deps option of [env:] section: midilab/uClock@^2.3.0
```ini
[env:...]
lib_deps =
    midilab/uClock@^2.3.0
```
3. Build a project, PlatformIO will automatically install dependencies.

### Arduino IDE
1. Open your Arduino IDE
2. Select the Library Manager Tab on left side
3. Type "uclock" at the search box
4. Click Install for latest version

## Core Concepts

### Clock Resolutions (PPQN)

PPQN (Pulses Per Quarter Note) determines the timing resolution of your clock:

- **PPQN_1, 2, 4, 8, 12**: Modular synthesis sync standards
- **PPQN_24**: Standard MIDI sync (24 pulses per beat)
- **PPQN_48**: Common in vintage drum machines
- **PPQN_96**: High-resolution internal clock (default)
- **PPQN_384, 480, 960**: Ultra-high resolution for modern sequencing and DAWs

### Callback Architecture

uClock operates through a callback system that triggers your code at precise intervals:

| Callback | Purpose | Use Case |
|----------|---------|----------|
| `setOnOutputPPQN()` | Main clock pulse | Drive sequencers, process MIDI |
| `setOnStep()` | 16th note intervals | Step sequencers, drum patterns |
| `setOnSync()` | Custom sync outputs | Multiple device sync, modular CV |
| `setOnClockStart()` | Clock start event | Initialize sequences, send MIDI start |
| `setOnClockStop()` | Clock stop event | Reset states, send MIDI stop |
| `setOnClockPause()` | Clock pause event | Pause handling |
| `setOnClockContinue()` | Clock continue event | Resume from pause |

## Quick Start

### Basic 96 PPQN Clock

```cpp
#include <uClock.h>

void onPPQNCallback(uint32_t tick) {
    // Called at each clock pulse (default 96 PPQN)
    // Drive your sequencer logic here
}

void setup() {
    // set main clock rate for output(sequencer resolution)
    uClock.setOutputPPQN(uClock.PPQN_96);
    // Configure callbacks
    uClock.setOnOutputPPQN(onPPQNCallback);

    // Set tempo
    uClock.setTempo(120.0);

    // Initialize and start
    uClock.init();
    uClock.start();
}

void loop() {
    // Your main code here
}
```

### Basic MIDI Clock box with External Sync
Send clock message and sync to external midi clock.

```cpp
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

  // set main clock rate for output(sequencer resolution)
  uClock.setOutputPPQN(uClock.PPQN_96);
  // set main clock rate for output(sequencer resolution) and input(expected sync signal)
  uClock.setInputPPQN(uClock.PPQN_24);
  
  // Set the callback function for the clock output to send MIDI Sync message based on 24PPQN
  uClock.setOnSync(uClock.PPQN_24, onSync24Callback);
  
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStartOutput(onClockStart);
  uClock.setOnClockStopOutput(onClockStop);

  // Inits the clock
  uClock.init();

  // set external clock mode
  uClock.setClockMode(uClock.EXTERNAL_CLOCK);

  // Set the clock BPM to 126 BPM
  //uClock.setTempo(126);
  // Starts the clock, tick-tac-tick-tac...
  //uClock.start();
}

void loop() {
  // call clockMe() each time you receive an external clock pulse
  // in this example we set inputPPQN to 24 PPQN
  // wich expects a signal comming from MIDI device
  if (Serial.available() > 0) {
    uint8_t midi_byte = Serial.read();
    
    switch (midi_byte) {
        case MIDI_CLOCK:
            uClock.clockMe();
            break;
    
        case MIDI_START:
            uClock.start();  // Let uClock handle start
            break;
    
        case MIDI_STOP:
            uClock.stop();   // Let uClock handle stop
            break;
    
        // Optional: ignore other real-time messages
        // case 0xF9: // Tick
        // case 0xFE: // Active Sense
        //     break;
    }
  }
}
```

## Advanced Features

### Multiple Sync Outputs

Generate different clock resolutions simultaneously for various devices:

```cpp
#include <uClock.h>

#define SYNC_OUT_PIN 8
#define MIDI_CLOCK_BYTE 0xF8

void onSync1(uint32_t tick) {
    // Send modular sync (1 pulse per quarter note)
    triggerModularPulse();
}

void onSync24(uint32_t tick) {
    // Send MIDI clock (24 PPQN)
    Serial.write(MIDI_CLOCK_BYTE);
}

void onSync48(uint32_t tick) {
    // Send 48 PPQN for vintage gear
    digitalWrite(SYNC_OUT_PIN, !digitalRead(SYNC_OUT_PIN));
}

void setup() {    
    // set main clock rate for output(sequencer resolution) and input(expected sync signal)
    uClock.setOutputPPQN(uClock.PPQN_96);
    // set sync callbacks
    uClock.setOnSync(uClock.PPQN_1, onSync1);
    uClock.setOnSync(uClock.PPQN_24, onSync24);
    uClock.setOnSync(uClock.PPQN_48, onSync48);
    // do only init after all setup is done
    uClock.init();
    
    // Set the clock BPM to 126 BPM
    uClock.setTempo(126);
    uClock.start();
}
```

**Available Sync Resolutions**: All possible Clock Resolutions (PPQN) where setOnSync(resolution) <= setOutputPPQN(resolution)

### Step Sequencer Extension

uClock includes a built-in extension specifically designed for creating step sequencers for synthesizers and drum machines. The extension provides multi-track support with independent per-track control, making it ideal for building complete rhythm machines and melodic sequencers.

#### Features

**Current Features**:
- ✅ **16th note orientation**: Natural step sequencer workflow
- ✅ **Multi-track support**: Independent sequences with individual callbacks
- ✅ **Per-track shuffle**: Each track can have its own groove templatexs

**Roadmap**:
- 🔄 **Per-track shift**: Offset patterns in time (coming soon)
- 🔄 **Per-track direction**: Forward, reverse, ping-pong playback (coming soon)

```cpp
#include <uClock.h>

// Called every 16th note
// not dependent on internal or external clock resolution
// single track callback(doesn't mean you can't code a multirack sequencer here)
void onStepCallback(uint32_t step) {
    // triger synth or drum notes
}

// the multirack callback 
//void onStepCallback(uint32_t step, uint8_t track) {
//}

void setup() {
    // Configure callbacks
    uClock.setOnStep(onStepCallback);
    // want multitrack support on shuffle per track?
    //uClock.setOnStep(onStepCallback, 8);
    
    uClock.init();
    
    uClock.start();
}
```

### Shuffle and Groove

Add humanization and swing to your sequences with shuffle support. Re-create timeless groove signatures of legends like MPC60 or TR-909, or experiment with your own custom groove templates.

#### How Shuffle Works

![Ableton Shuffle Example](https://raw.githubusercontent.com/midilab/uclock/develop/doc/shuflle-example.gif)

Shuffle operates by shifting individual steps earlier or later in time, along with adjusting note lengths to maintain musical coherence. As shown in the Ableton example above:

- **Positive shuffle values**: Delay the note trigger and shorten note length (to avoid overlapping with the next note)
- **Negative shuffle values**: Trigger the note earlier and extend note length (filling the time gap)
- **Zero values**: Play straight with no shuffle effect

This is the fundamental principle behind groove templating in step sequencers. Each 16th note step can be individually time-shifted to create swing, shuffle, or completely custom rhythmic feels.

#### Technical Specifications

**Shuffle templates are PPQN-dependent**: The range of shuffle values depends on your output resolution:

```cpp
// Shuffle range calculation - in ticks
min_shuffle = -(output_ppqn/4) - 1
max_shuffle = +(output_ppqn/4) - 1

// Example with PPQN_96 (default)
// Range: -23 to +23 ticks per step
```

**Template Size Configuration**: Adjust in `uClock.h` if needed:

```cpp
#define MAX_SHUFFLE_TEMPLATE_SIZE   16
```

Modify this value to support longer patterns (memory permitting) or reduce it to save memory if you only need simpler grooves.

MPC60 Groove implmentation example based on [Roger Linn Groove Magic Interview](https://www.attackmagazine.com/features/interview/roger-linn-swing-groove-magic-mpc-timing/)

```cpp
#include <uClock.h>

#define TRACKS_SIZE 8

// The shuffle effect is applied transparently for shuffled steps
void onStepCallback(uint32_t step, uint8_t track) {
    // get the length diff to apply to current shuffled step length
    // if the step is not shuffled, then shuffle_len == 0
    int8_t shuffle_len = uClock.getShuffleLength(track);
    // sequencer procesing...
    // ...
    uint8_t step_len = getNoteLength(step, track);
    step_len += shuffle_len;
    // use step_len as the new length of note for this step
    // ...
}

// global shuffle instead?
//void onStepCallback(uint32_t step) {
//}

void setup() {
    // MPC60 groove signature (Same used for TR909)
    // Based on Roger Linn Groove Magic Interview
    // Internal clock: 96 PPQN
    // Each step is 24 ticks long
    uint8_t current_shuffle = 0;
    int8_t shuffle_50[2] = {0, 0};
    int8_t shuffle_54[2] = {0, 2};
    int8_t shuffle_58[2] = {0, 4};
    int8_t shuffle_62[2] = {0, 6};
    int8_t shuffle_66[2] = {0, 8};
    int8_t shuffle_71[2] = {0, 10};
    int8_t shuffle_75[2] = {0, 12};
    String shuffle_name[7] = {"50%", "54%", "58%", "62%", "66%", "71%", "75%"};
    int8_t* shuffle_templates[7] = {shuffle_50, shuffle_54, shuffle_58, shuffle_62, shuffle_66, shuffle_71, shuffle_75};
    
    // set main clock rate for output(sequencer resolution) and input(expected sync signal)
    uClock.setOutputPPQN(uClock.PPQN_96);
    // passing second argument for the number of tracks to handle
    // multitrack individual shuffle support
    uClock.setOnStep(onStepCallback, TRACKS_SIZE);
    // global shuffle? no need to multitrack handle of step sequencer extension
    //uClock.setOnStep(onStepCallback);
    
    uClock.init();
    
    // set a template for shuffle
    uClock.setShuffleTemplate(shuffle_templates[current_shuffle]);
    // enable/disable shuffle
    uClock.setShuffle(true);
    
    uClock.start();
}
```

#### Shuffle API Reference

```cpp
// Enable/disable shuffle for a track
void setShuffle(bool active, uint8_t track = 0);

// Check if shuffle is active
bool isShuffled(uint8_t track = 0);

// Set shuffle pattern size (max 16 steps)
void setShuffleSize(uint8_t size, uint8_t track = 0);

// Set individual shuffle values
void setShuffleData(uint8_t step, int8_t tick, uint8_t track = 0);

// Set complete shuffle template
void setShuffleTemplate(int8_t* shuff, uint8_t size, uint8_t track = 0);

// Get shuffle offset for note length compensation
int8_t getShuffleLength(uint8_t track = 0);
```

**Shuffle Values**:
- Range: `-(output_ppqn/4)-1` to `+(output_ppqn/4)-1` ticks
- Negative values = play earlier
- Positive values = delay
- Zero = no shuffle (straight timing)

### External Sync Phase Lock

Fine-tune synchronization with external clocks:

```cpp
void setup() {
    // sets external clock
    uClock.setClockMode(uClock.EXTERNAL_CLOCK);
    // get back to internal clock
    //uClock.setClockMode(uClock.INTERNAL_CLOCK);

    // Lock phase every N quarters (default: 1)
    // Higher values = looser sync but more stable
    // Lower values = tighter sync but may jitter
    uClock.setPhaseLockQuartersCount(1);

    // Smooth tempo reading for display
    // Buffer size: 1-254 (larger = smoother but slower lock time response)
    // The higher the value the longer it takes to visual bpm sync
    uClock.setExtIntervalBuffer(64);
}
```

## Configuration API

### Clock Control

```cpp
// Set internal tempo (1-500 BPM)
void setTempo(float bpm);

// Get current tempo (works with external sync too)
float getTempo();

// Clock transport control
void start();        // Start clock
void stop();         // Stop clock
void pause();        // Toggle pause/continue

// Clock mode
void setClockMode(ClockMode mode);
ClockMode getClockMode();
// Modes: INTERNAL_CLOCK, EXTERNAL_CLOCK
```

### Resolution Configuration

```cpp
// Set main output resolution
void setOutputPPQN(PPQNResolution resolution);

// Set expected input resolution (external sync)
void setInputPPQN(PPQNResolution resolution);

// Available resolutions:
// PPQN_1, PPQN_2, PPQN_4, PPQN_8, PPQN_12, PPQN_24
// PPQN_48, PPQN_96, PPQN_384, PPQN_480, PPQN_960
```

**Important**: `inputPPQN` must be ≤ `outputPPQN`

### Timing Utilities

```cpp
// Convert BPM to microseconds
uint32_t bpmToMicroSeconds(float bpm);

// Elapsed time functions
uint8_t getNumberOfSeconds(uint32_t time);
uint8_t getNumberOfMinutes(uint32_t time);
uint8_t getNumberOfHours(uint32_t time);
uint8_t getNumberOfDays(uint32_t time);

// Timers
uint32_t getNowTimer();
uint32_t getPlayTime();
```

## Software Timer Mode

For unsupported boards or to avoid interrupt conflicts, uClock can run in software mode:

**Automatic Fallback**: Activates automatically on unsupported boards

**Manual Activation**: Define `USE_UCLOCK_SOFTWARE_TIMER` build flag

```cpp
void loop() {
    // REQUIRED in software timer mode
    uClock.run();

    // Part of your code here
    // ...
    
    // Call run() frequently for best accuracy
    uClock.run();

    // Other parts of your code here
    // ...

    uClock.run();
}
```

⚠️ **Note**: Software timer mode provides less accurate timing than hardware interrupts.

## Migration Guide (v1.x → v2.0)

### Breaking Changes

| Old API (v1.x) | New API (v2.0+) |
|----------------|-----------------|
| `setClock96PPQNOutput()` | `setOnOutputPPQN()` |
| `setClock16PPQNOutput()` | `setOnStep()` |
| `setOnClockStartOutput()` | `setOnClockStart()` |
| `setOnClockStopOutput()` | `setOnClockStop()` |

### Resolution Changes

v2.0 introduces flexible PPQN configuration. If you relied on the old 96 PPQN default:

```cpp
// Old (implicit 96 PPQN)
uClock.setClock96PPQNOutput(callback);

// New (explicit configuration)
uClock.setOutputPPQN(uClock.PPQN_96);
uClock.setOnOutputPPQN(callback);
```

### Tick Counting

If you used `setClock16PPQNOutput()` (16th notes), simply replace with `setOnStep()` - no other changes needed.

If you relied on specific tick values from `setClock96PPQNOutput()`, verify your timing calculations match your chosen PPQN resolution.

## Examples

Complete examples are available in the `examples/` directory:

- **BasicClock**: Simple internal clock setup
- **ExternalSync**: Slave to external clock
- **MidiClockSync**: Full MIDI sync box implementation
- **AcidStepSequencer**: TB-303 style sequencer engine
- **MultiTrackSequencer**: Independent track sequencing with shuffle

## Technical Details

### Thread Safety

All clock operations that have shared variables or resources migth use atomic operations. When modifying shared variables from your code:

```cpp

ATOMIC(
    // Modify shared variables safely
    tempo = newTempo;
)
```

## License

MIT License - Copyright (c) 2024 Romulo Silva

## Support & Community

- **Documentation**: [GitHub Repository](https://github.com/midilab/uClock)
- **Issues**: Report bugs via GitHub Issues
- **Contact**: contact@midilab.co

## Acknowledgments

Created and maintained by Romulo Silva for the open-source music technology community.

---

**uClock** - Precision timing for creative minds.
