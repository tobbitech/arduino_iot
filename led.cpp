#include <Arduino.h>
#include <FastLED.h>
#include "led.h"




WS2812B_Led::WS2812B_Led() {
    //_pin = pin;
    //_num_leds = num_leds;
}

void WS2812B_Led::begin() {

    //FastLED.addLeds<WS2812B, pin, GRB>(_leds, _num_leds).setCorrection( TypicalSMD5050 );
    FastLED.addLeds<CHIPSET, LEDPIN, COLOR_ORDER>(_leds, NUM_LEDS).setCorrection( TypicalSMD5050 );
    FastLED.setBrightness(255);

    for (int i = 0; i < NUM_LEDS; i++) {
        _leds[i] = CRGB::Black;
        _blink_start_time[i] = 0;
        _blink_on_duration[i] = 0;
        _blink_off_duration[i] = 0;
        _blink_times[i] = 0;
        _blink_color[i] = CRGB::Black;
    }

}

void WS2812B_Led::tick() {
    for (int i = 0; i < NUM_LEDS; i++) {
        if (_blink_times[i] > 0) {
            

            if (_leds[i] != CRGB::Black) {
                if (millis() - _blink_start_time[i] > _blink_on_duration[i]) {
                    Serial.print("Blinking LED ");
                    Serial.print(i + 1);
                    Serial.println(" OFF");
                    _leds[i] = CRGB::Black;
                    FastLED.show();
                    _blink_start_time[i] = millis();
                    _blink_times[i]--;
                }
            } else {
                if (millis() - _blink_start_time[i] > _blink_off_duration[i]) {
                    Serial.print("Blinking LED ");
                    Serial.print(i + 1);
                    Serial.println(" ON");
                    _leds[i] = _blink_color[i];
                    FastLED.show();
                    _blink_start_time[i] = millis();
                }
            }
        }
    }
}

void WS2812B_Led::on(int led_no, CRGB color) {
    if (led_no < 1 || led_no > NUM_LEDS) {
        Serial.println("Invalid led number");
        return;
    }
    _leds[led_no - 1] = color;
    FastLED.show();
}

void WS2812B_Led::on(int led_no, CRGB color, u_int8_t brightness) {
    WS2812B_Led::on(led_no, color);
    FastLED.setBrightness(brightness);
}

void WS2812B_Led::blink(u_int8_t led_no, CRGB color, u_int32_t on_duration_ms, u_int32_t off_duration_ms, u_int8_t times) {
    if (led_no < 1 || led_no > NUM_LEDS) {
        Serial.println("Invalid led number");
        return;
    }
    _blink_start_time[led_no - 1] = millis();
    _blink_on_duration[led_no - 1] = on_duration_ms;
    _blink_off_duration[led_no - 1] = off_duration_ms;
    _blink_times[led_no - 1] = times + 1; // fix for not ending with LED on
    _blink_color[led_no - 1] = color;

}

void WS2812B_Led::blink_blocking(u_int8_t led_no, CRGB color, u_int32_t on_duration_ms, u_int32_t off_duration_ms, u_int8_t times) {
    if (led_no < 1 || led_no > NUM_LEDS) {
        Serial.println("Invalid led number");
        return;
    }
    for (int i = 0; i < times; i++) {
        _leds[led_no - 1] = color;
        FastLED.show();
        delay(on_duration_ms);
        _leds[led_no - 1] = CRGB::Black;
        FastLED.show();
        if (i < times - 1) {
            delay(off_duration_ms);
        }
    }

}


void WS2812B_Led::off(int led_no) {
    if (led_no < 1 || led_no > NUM_LEDS) {
        Serial.println("Invalid led number");
        return;
    }
    _leds[led_no - 1 ] = CRGB::Black;
    FastLED.show();
}

void WS2812B_Led::set_brightness(int brightness) {
    FastLED.setBrightness(brightness);
    FastLED.show();
}

void WS2812B_Led::print_leds() {
    for (int i = 0; i < NUM_LEDS; i++) {
        Serial.print("LED ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(_leds[i].r);
        Serial.print(", ");
        Serial.print(_leds[i].g);
        Serial.print(", ");
        Serial.print(_leds[i].b);
        Serial.print("\t\t");
    }
    Serial.println();
}

void WS2812B_Led::toggle(int led_no, CRGB color) {
    if (led_no < 1 || led_no > NUM_LEDS) {
        Serial.println("Invalid led number");
        return;
    }
    if (_leds[led_no - 1] == CRGB::Black) {
        _leds[led_no - 1] = color;;
    } else {
        _leds[led_no - 1] = CRGB::Black;
    }
    FastLED.show();
}