#pragma once

#include "mavesp8266.h"
#include <FastLED.h>

class MavESP8266Neopixel : public MavESP8266Leds {
public:
    MavESP8266Neopixel();
    void begin() override;
    void fill_leds(uint8_t r, uint8_t g, uint8_t b);
    void set_brightness(uint8_t brightness);
private:
    static constexpr uint8_t data_pin_ = 12;
    static constexpr uint8_t num_leds_ = 64;
    const uint8_t brightness_ = 10;

    CRGB leds[num_leds_];
};