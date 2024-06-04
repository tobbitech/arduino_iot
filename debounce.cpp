#include <Arduino.h>
#include "debounce.h"


DebounceButton::DebounceButton(int pin, u_int32_t debounce_delay) {
    _pin = pin;
    _debounce_delay = debounce_delay;
    _state = DebounceButton::RESET;
    _last_state = DebounceButton::RESET;
    _last_debounce_time = 0;
    _is_pressed = false;
    _is_released = false;
    _sticky_timer.set(0, "seconds");
    _is_sticky_held = false;

}

void DebounceButton::set_sticky_button_timer(Timer sticky_timer) {
    _sticky_timer = sticky_timer;
}

uint32_t DebounceButton::get_remaining_sticky_hold_time_ms() {
    return(_sticky_timer.remaining() );
}

bool DebounceButton::is_sticky_held() {
    return (_is_sticky_held);
}

bool DebounceButton::is_pressed() {
    return _is_pressed;
}

bool DebounceButton::is_released() {
    return _is_released;
}

bool DebounceButton::is_held() {
    return _is_held;
}

u_int32_t DebounceButton::get_hold_time_ms() {
    return millis() -_hold_time_ms;
}

void DebounceButton::tick() {
    switch_value = digitalRead(_pin);
    _last_state = _state;

    if (_sticky_timer.is_done() ) {
        _is_sticky_held = false;
    }


    switch(_state) {
        case DebounceButton::RESET:
            _is_pressed = false;
            _is_held = false;
            _is_released = false;
            _hold_time_ms = millis();
            _state = DebounceButton::START;
            break;
        case DebounceButton::START:
            if (switch_value == LOW) {
                _state = DebounceButton::GO;
            } else {
                _state = DebounceButton::RESET;
            }
            break;
        case DebounceButton::GO:
            _debounce_timer.set(_debounce_delay, "milliseconds");
            _state = DebounceButton::WAIT;
            break;
        case DebounceButton::WAIT:
            if (switch_value == HIGH) {
                _state = DebounceButton::RESET;
            } else if (_debounce_timer.is_done()) {
                _state = DebounceButton::TRIGGERED;
            }
            break;
        case DebounceButton::TRIGGERED:
            _state = DebounceButton::HELD;
            _is_pressed = true;
            _hold_time_ms = millis();
            // Serial.println("Button pressed");
            // start sticky button timer
            _sticky_timer.reset();
            _is_sticky_held = true;
            break;

        case DebounceButton::HELD:
            _is_pressed = false;
            _is_held = true;
            if (switch_value == HIGH) {
                _state = DebounceButton::RELEASED;
            }
            break;
        case DebounceButton::RELEASED:
            _is_held = false;
            _is_released = true;
            _state = DebounceButton::RESET;
            break;
    }   
}