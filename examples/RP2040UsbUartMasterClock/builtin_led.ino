#if defined(WS2812_BUILTIN_LED)
#include <Adafruit_NeoPixel.h>
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, WS2812_BUILTIN_LED, NEO_GRB + NEO_KHZ800);
#endif

// check the pinage for BUILTIN LED of your model in case LED_BUILTIN wont ligth up
// this is valid only if you're not using rgb version ws2812 (WS2812_BUILTIN_LED)
//#define LED_BUILTIN PIN_LED_B

void initBlinkLed() {
#if defined(WS2812_BUILTIN_LED)
  // use adafruit neo pixel 
  pixels.begin();
#else
  // normal led pin
  pinMode(LED_BUILTIN, OUTPUT);
#endif
}

void ledOn() {
#if defined(WS2812_BUILTIN_LED)
  pixels.setPixelColor(0, pixels.Color(0, 0, 20));
  pixels.show();  // turn the LED on (HIGH is the voltage level)
#else
  digitalWrite(LED_BUILTIN, LOW);
#endif
}

void ledOff() {
#if defined(WS2812_BUILTIN_LED)
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
#else
  digitalWrite(LED_BUILTIN, HIGH);
#endif
}