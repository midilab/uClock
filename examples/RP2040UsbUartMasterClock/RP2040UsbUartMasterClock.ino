/* 
 * USB/Uart MIDI Sync Box 
 *  
 * This example code is in the public domain.
 * 
 */

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#include <uClock.h>

// Instantiate the MIDI interfaces
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

// Do your rpi 2040 has a ws2812 RGB LED? set the pin!
// otherwise keep it commented for normal LED_BUILTIN led blinking
#define WS2812_BUILTIN_LED  16

uint8_t bpm_blink_timer = 1;
void handle_bpm_led(uint32_t tick)
{
  // BPM led indicator
  if ( !(tick % (96)) || (tick == 1) ) {  // first of 4 quarter pulse will flash longer
    bpm_blink_timer = 8;
    ledOn();
  } else if ( !(tick % (24)) ) {   // each quarter led on
    bpm_blink_timer = 1;
    ledOn();
  } else if ( !(tick % bpm_blink_timer) ) { // get led off
    ledOff();
  }
}

// Internal clock handlers
void onSync24Callback(uint32_t tick) {
  // Send MIDI_CLOCK to external gears
  MIDI.sendRealTime(midi::Clock);
  MIDI_USB.sendRealTime(midi::Clock);
  // blink tempo
  handle_bpm_led(tick);
}

void onClockStart() {
  MIDI.sendRealTime(midi::Start);
  MIDI_USB.sendRealTime(midi::Start);
}

void onClockStop() {
  MIDI.sendRealTime(midi::Stop);
  MIDI_USB.sendRealTime(midi::Stop);
}

void setup() {
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif

  // Initialize USB midi stack
  MIDI_USB.begin(MIDI_CHANNEL_OMNI);
  // Initialize UART midi stack
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Initialize builtin led for clock timer blinking
  initBlinkLed();

  // Setup our clock system
  // Inits the clock
  uClock.init();
  // Set the callback function for the clock output to send MIDI Sync message.
  uClock.setOnSync24(onSync24Callback);
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStart(onClockStart);  
  uClock.setOnClockStop(onClockStop);
  // Set the clock BPM to 126 BPM
  uClock.setTempo(126);
  // Starts the clock, tick-tac-tick-tac..
  uClock.start();
}

// Do it whatever to interface with Clock.stop(), Clock.start(), Clock.setTempo() and integrate your environment...
void loop() {
  // handle midi input?
  MIDI.read();
  MIDI_USB.read();
}
