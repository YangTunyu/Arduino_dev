#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT  8
#define LEDS_PIN    13
#define CHANNEL     0

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

enum Mode { GRADIENT, JUMP, FLASH, RAINBOW, BREATH };
Mode mode = GRADIENT;

void setup() {
  strip.begin();
  strip.setBrightness(20);  
}

void loop() {
  switch(mode) {
    case GRADIENT:
      for (int j = 0; j < 255; j += 2) {
        for (int i = 0; i < LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel((i * 256 / LEDS_COUNT + j) & 255));
        }
        strip.show();
        delay(10);
      }
      delay(2000);  // 2 seconds delay
      mode = JUMP;
      break;
    case JUMP:
      for (int i = 0; i < LEDS_COUNT; i++) {
        strip.setLedColorData(i, strip.Wheel(random(0, 255)));
      }
      strip.show();
      delay(2000);  // 2 seconds delay
      mode = FLASH;
      break;
    case FLASH:
      for (int i = 0; i < LEDS_COUNT; i++) {
        strip.setLedColorData(i, strip.Wheel(random(0, 255)));
      }
      strip.show();
      delay(100);
      for (int i = 0; i < LEDS_COUNT; i++) {
        strip.setLedColorData(i, 0);
      }
      strip.show();
      delay(100);
      delay(2000);  // 2 seconds delay
      mode = RAINBOW;
      break;
    case RAINBOW:
      for (int j = 0; j < 256; j++) {
        for (int i = 0; i < LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel((i + j) & 255));
        }
        strip.show();
        delay(10);
      }
      delay(2000);  // 2 seconds delay
      mode = BREATH;
      break;
    case BREATH:
      for (int j = 0; j < 255; j++) {
        for (int i = 0; i < LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel(j));
        }
        strip.show();
        delay(10);
      }
      for (int j = 255; j > 0; j--) {
        for (int i = 0; i < LEDS_COUNT; i++) {
          strip.setLedColorData(i, strip.Wheel(j));
        }
        strip.show();
        delay(10);
      }
      delay(2000);  // 2 seconds delay
      mode = GRADIENT;
      break;
  }
}
