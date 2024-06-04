#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include <FastLED.h>

#define LEDPIN 4
#define NUM_LEDS 3
#define CHIPSET WS2812B
#define COLOR_ORDER GRB

class WS2812B_Led {
    public:
        WS2812B_Led();
        void begin();
        void tick();
        void on(int led_no, CRGB color);
        void on(int led_no, CRGB color, u_int8_t brightness);
        void on_for(int led_no, CRGB color, int duration_ms);
        void off(int led_no);
        void set_brightness(int brightness);
        void toggle(int led_no, CRGB color);
        void blink(u_int8_t led_no, CRGB color, u_int32_t on_duration_ms, u_int32_t off_duration_ms, u_int8_t times);
        void blink_blocking(u_int8_t led_no, CRGB color, u_int32_t on_duration_ms, u_int32_t off_duration_ms, u_int8_t times);
        void print_leds();
    private:
        //int _pin;
        //int _num_leds;
        CRGB _leds[NUM_LEDS];
        u_int32_t _blink_start_time[NUM_LEDS];
        u_int32_t _blink_on_duration[NUM_LEDS];
        u_int32_t _blink_off_duration[NUM_LEDS];
        u_int8_t _blink_times[NUM_LEDS];
        CRGB _blink_color[NUM_LEDS];



};  




#endif