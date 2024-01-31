/* USB MIDI Sync Box - example that also prints to serial and sends USB midi as well as blinking LED
 * (usb part still needs testing to make sure it works!)
 *  
 * This example code is in the public domain.
 * 
 */
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);

//#define LED_BUILTIN PIN_LED_B

//MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
#include <uClock.h>

uint8_t bpm_blink_timer = 1;
void handle_bpm_led(uint32_t tick)
{
  // BPM led indicator
  if ( !(tick % (96)) || (tick == 1) ) {  // first compass step will flash longer
    bpm_blink_timer = 8;
    digitalWrite(LED_BUILTIN, LOW);
  } else if ( !(tick % (24)) ) {   // each quarter led on
    bpm_blink_timer = 1;
    digitalWrite(LED_BUILTIN, LOW);
  } else if ( !(tick % bpm_blink_timer) ) { // get led off
    digitalWrite(LED_BUILTIN, HIGH);
  }
}


// Internal clock handlers
void onSync24Callback(uint32_t tick) {
  // Send MIDI_CLOCK to external gears
  MIDI_USB.sendRealTime(midi::Clock);
  handle_bpm_led(tick);
}

void onClockStart() {
  MIDI_USB.sendRealTime(midi::Start);
}

void onClockStop() {
  MIDI_USB.sendRealTime(midi::Stop);
}

void setup() {
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif

  MIDI_USB.begin(MIDI_CHANNEL_OMNI);

  // A led to count bpms
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  while (!Serial)
    delay(1);
  
  // wait until device mounted
  /*while( !TinyUSBDevice.mounted() ) {
    Serial.println("waiting for usb..");
    Serial.flush();
    delay(1);
  }*/

  // Setup our clock system
  // Inits the clock
  Serial.println("about to uClock.init()..."); Serial.flush();
  uClock.init();
  // Set the callback function for the clock output to send MIDI Sync message.
  uClock.setOnSync24(onSync24Callback);
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStart(onClockStart);  
  uClock.setOnClockStop(onClockStop);
  // Set the clock BPM to 126 BPM
  uClock.setTempo(126);
  // Starts the clock, tick-tac-tick-tac..
  Serial.println("about to uClock.start()..."); Serial.flush();
  uClock.start();
  Serial.println("uClock.start()ed!"); Serial.flush();
}

uint32_t count = 0;

// Do it whatever to interface with Clock.stop(), Clock.start(), Clock.setTempo() and integrate your environment...
void loop() {
  MIDI_USB.read();
  count++;
  if (millis()%1000==0)
    Serial.println("looped!!!");
}
