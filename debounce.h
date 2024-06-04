#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <Arduino.h>
#include "timer.h"

class DebounceButton {
    public:
        DebounceButton(int pin, u_int32_t debounce_delay = 50);
        void tick();
        bool is_pressed();
        bool is_held();
        bool is_released();
        u_int32_t get_hold_time_ms();

        enum state {
            RESET,
            START,
            GO,
            WAIT,
            TRIGGERED,
            HELD,
            RELEASED
        };



    private:
        int _pin;
        int _state;
        int _last_state;
        u_int32_t _last_debounce_time;
        u_int32_t _debounce_delay;
        u_int32_t _hold_time_ms;
        Timer _debounce_timer;

        int switch_value;
        bool _is_pressed;
        bool _is_held;
        bool _is_released; 
};

#endif