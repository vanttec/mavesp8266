#include "mavesp8266_neopixel.h"

MavESP8266Neopixel::MavESP8266Neopixel() {
    ;
}

void MavESP8266Neopixel::begin() {
    FastLED.addLeds<WS2812B, data_pin_, GRB>(leds, num_leds_).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(brightness_);
    fill_leds(0, 0, 0);
}

void MavESP8266Neopixel::fill_leds(uint8_t r, uint8_t g, uint8_t b){
    CRGB color;
    color.setRGB(r, g, b);
    fill_solid(leds, num_leds_, color);
    FastLED.show();
}