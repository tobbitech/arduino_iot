#ifndef IOT_CAPABILITY_H
#define IOT_CAPABILITY_H

#include <Arduino.h>
#include "mqttConnection.h"
#include "debounce.h"

class OnOffSwitch
{
    public:
        OnOffSwitch(Connection * conn_pointer, int pin, String name, String mqtt_topic, String on_value = "true", String off_value = "false");
        //void OnOffSwitch(Connection conn, int pin, String name, String mqtt_topic, String on_value = "true", String off_value = "false"); 

        void begin();
        String getMqttTopic();
        String getName();
        void turnOn(bool updateOnOfTopic = false);
        void turnOff(bool updateOnOfTopic = false);
        void toggle(bool updateOnOfTopic = false);
        void setSwitchState(String on_off_value, bool updateOnOfTopic = false);
        String getOnValue();
        String getOffValue();


    private:
        Connection * _conn_pointer;
        int _pin;
        String _name;
        String _mqtt_topic;
        String _mac_address;
        String _on_value;
        String _off_value;
};

class DS18B20_temperature_sensors
{
    public:
        DS18B20_temperature_sensors(Connection * conn_pointer, int pin, String mqtt_main_topic);
        String getMqttTopic();
        String getName(int deviceIndex);
        String convertAddressToString(DeviceAddress address);
        String getAddressString(int deviceIndex);
        uint8_t scanForSensors();
        float getTemperature(int deviceIndex);
        void mapNameToDeviceAddress(DeviceAddress address, String name);
        void publishAllTemperatures();

    private:
        Connection * _conn_pointer;
        OneWire _oneWire;
        DallasTemperature _sensors;
        DeviceAddress _deviceAddresses[127];
        String _deviceNames[127];
        uint8_t _numberOfDevices;
        String _name;
        String _mqtt_main_topic;
        String _addressMap[127];
        String _nameMap[127];
        size_t _mapSize;
};


#define INPUT_MOMENTARY_HIGH_ON 0
#define INPUT_MOMENTARY_LOW_ON  1
#define INPUT_MOMENTARY_ANALOG  3
class InputMomentary
{
    public:
        InputMomentary(
                Connection * conn_pointer, 
                int pin, 
                String name, 
                String mqtt_topic, 
                uint8_t mode = INPUT_MOMENTARY_HIGH_ON,
                String on_value = "true", 
                String off_value = "false"
        );
        void begin();
        void check();
        void set_threshold_voltage(float voltage);



    private:
        Connection * _conn_pointer;
        int _pin;
        String _name;
        String _mqtt_topic;
        String _on_value;
        String _off_value;
        float _threshold_voltage;
        uint8_t _mode;
        bool _last_state;    
};

class DebounceButton {
    public:
        DebounceButton(
            Connection * conn_pointer, 
            int pin, 
            String name, 
            String mqtt_topic, 
            bool on_level = HIGH,
            u_int32_t debounce_delay = 50,
            String on_value = "true", 
            String off_value = "false");
        void tick();
        bool is_pressed();
        bool is_held();
        bool is_released();
        u_int32_t get_hold_time_ms();
        void set_sticky_button_timer(Timer sticky_timer);
        bool is_sticky_held();
        u_int32_t get_remaining_sticky_hold_time_ms();

        enum state {
            RESET,
            START,
            GO,
            WAIT,
            TRIGGERED,
            HELD,
            STICKY,
            RELEASED
        };



    private:
        int _pin;
        int _pressed;
        int _unpressed;
        int _state;
        int _last_state;
        u_int32_t _last_debounce_time;
        u_int32_t _debounce_delay;
        u_int32_t _hold_time_ms;
        Timer _debounce_timer;
        Timer _sticky_timer;

        int switch_value;
        bool _is_pressed;
        bool _is_held;
        bool _is_released; 
        bool _is_sticky_held;
};

// class DebounceInputMomentary 
// {
//     public:
//         DebounceInputMomentary(
//                 Connection * conn_pointer, 
//                 int pin, 
//                 String name, 
//                 String mqtt_topic, 
//                 uint8_t mode = INPUT_MOMENTARY_HIGH_ON,
//                 String on_value = "true", 
//                 String off_value = "false"
//         );
//         void begin();
//         void check();

//     private:
//         Connection * _conn_pointer;
//         int _pin;
//         String _name;
//         String _mqtt_topic;
//         String _on_value;
//         String _off_value;
//         DebounceButton _button;
// }


#define HAN_READ_TIMEOUT_MS 100
#define HAN_MAX_MESSAGE_SIZE 2000

class HANreader {
    public:
        HANreader(Connection * conn, String mqttTopic, uint8_t RXpin, uint8_t TXpin);
        void begin();
        void end();
        void tick();
        HardwareSerial serialHAN;
        void parse_message(String message);
        void parse_message();

        struct han_line {
            u_int8_t obis_code[6];
            String name;
            String unit;
            String topic;
        };

    private:
        Connection * _conn;
        String _mqttTopic;
        uint8_t _RXpin;
        uint8_t _TXpin;
        int16_t _state;
        int16_t _prev_state;
        char _recv_char;
        String _message;
        uint8_t _message_buf[HAN_MAX_MESSAGE_SIZE];
        uint16_t _message_buf_pos;
        void _receive_char();
        uint32_t _last_byte_millis;
        bool _match_sequence(uint16_t);
        u_int16_t _no_han_lines;
};

#define VEDIRECT_TIMEOUT_MS 100
#define VEDIRECT_MESSAGE_SIZE 2000
#define VEDIRECT_NUMBER_KEYS_TO_PARSE 20
class VEdirectReader {
    public:
        VEdirectReader(Connection *conn, String mqttTopic, u_int8_t RXpin, u_int8_t TXpin);
        void begin();
        void end();
        void tick();
        HardwareSerial serialVE;
        void parse_message();
        void set_publish_timer_s(u_int16_t seconds);
        void publish_data();

    private:
        Connection * _conn;
        String _mqttTopic;
        uint8_t _RXpin;
        uint8_t _TXpin;
        int16_t _state;
        int16_t _prev_state;
        char _recv_char;
        String _message;
        uint8_t _message_buf[VEDIRECT_MESSAGE_SIZE];
        uint16_t _message_buf_pos;
        void _receive_char();
        uint32_t _last_byte_millis;
        Timer _send_raw_data_timer;
        Timer _publish_data_timer;
        float _voltage_V;
        float _current_A;
        float _power_W;
        float _soc;
        float _soc_by_v;
        float _pv_voltage_V;
        float _pv_power_W;
        float _yield_total_kWh;
        float _yield_today_kWh;
        float _max_power_today_W;
        float _yield_yesterday_kWh;
        float _max_power_yesterday_W;
        bool _voltage_is_set;
        bool _current_is_set;
        bool _power_is_set;
        bool _soc_is_set;
        bool _pv_voltage_is_set;
        bool _pv_power_is_set;
        bool _yield_total_is_set;
        bool _yield_today_is_set;
        bool _max_power_today_is_set;
        bool _yield_yesterday_is_set;
        bool _max_power_yesterday_is_set;
};

#endif