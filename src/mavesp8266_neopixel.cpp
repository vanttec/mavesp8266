#include "mavesp8266_neopixel.h"
#include "mavesp8266_parameters.h"

MavESP8266Neopixel::MavESP8266Neopixel() {
    ;
}

void MavESP8266Neopixel::begin() {
    FastLED.addLeds<WS2812B, data_pin_, GRB>(leds, num_leds_).setCorrection(TypicalSMD5050);

    // Low brightness red blink on turn on.
    set_brightness(255);
    fill_leds(0, 0, 0);
    delay(100);
    fill_leds(127, 0, 127);
    delay(100);

    set_brightness(getWorld()->getParameters()->getLedBrightness());
    fill_leds(0, 0, 0);
}

void MavESP8266Neopixel::fill_leds(uint8_t r, uint8_t g, uint8_t b){
    CRGB color;
    color.setRGB(r, g, b);
    fill_solid(leds, num_leds_, color);
    FastLED.show();
}

void MavESP8266Neopixel::set_brightness(uint8_t brightness){
    FastLED.setBrightness(brightness);
    FastLED.show();
}
