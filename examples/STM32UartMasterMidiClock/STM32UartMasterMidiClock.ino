/* Uart MIDI out
 *  
 * This example demonstrates how to send MIDI data via Uart 
 * interface on STM32 family. 
 * 
 * This example code is in the public domain.
 *
 * Requires STM32Duino board manager to be installed.
 * 
 * Define HardwareSerial using any available UART/USART. 
 * Nucleo boards have UART/USART pins that are used by the ST-LINK interface (unless using solder bridging).
 *
 * Tested on Nucleo-F401RE and Nucleo-F072RB (PA9=D8 PA10=D2 on the Arduino pins)
 *
 * Code by midilab contact@midilab.co
 * Example modified by Jackson Devices contact@jacksondevices.com
 */
#include <uClock.h>

// MIDI clock, start and stop byte definitions - based on MIDI 1.0 Standards.
#define MIDI_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP  0xFC

HardwareSerial Serial1(PA10, PA9);

uint8_t bpm_blink_timer = 1;
void handle_bpm_led(uint32_t tick)
{
  // BPM led indicator
  if ( !(tick % (96)) || (tick == 1) ) {  // first compass step will flash longer
    bpm_blink_timer = 8;
    digitalWrite(LED_BUILTIN, HIGH);
  } else if ( !(tick % (24)) ) {   // each quarter led on
    bpm_blink_timer = 1;
    digitalWrite(LED_BUILTIN, HIGH);
  } else if ( !(tick % bpm_blink_timer) ) { // get led off
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// Internal clock handlers
void onSync24Callback(uint32_t tick) {
  // Send MIDI_CLOCK to external gear
  Serial1.write(MIDI_CLOCK);
  handle_bpm_led(tick);
}

void onClockStart() {
    // Send MIDI_START to external gear
  Serial1.write(MIDI_START);
}

void onClockStop() {
    // Send MIDI_STOP to external gear
  Serial1.write(MIDI_STOP);
}

void setup() {
  // Initialize Serial1 communication at 31250 bits per second, the default MIDI Serial1 speed communication:
  Serial1.begin(31250);

  // An led to display BPM
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Setup our clock system
  // Inits the clock
  uClock.init();
  // Set the callback function for the clock output to send MIDI Sync message.
  uClock.setOnSync24(onSync24Callback);
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStart(onClockStart);  
  uClock.setOnClockStop(onClockStop);
  // Set the clock BPM to 126 BPM
  uClock.setTempo(120);
  // Starts the clock, tick-tac-tick-tac...
  uClock.start();
}

// Do it whatever to interface with Clock.stop(), Clock.start(), Clock.setTempo() and integrate your environment...
void loop() {
  
}
