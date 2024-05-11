#ifndef IOT_CAPABILITY_H
#define IOT_CAPABILITY_H

#include <Arduino.h>
#include "mqttConnection.h"
#include <SoftwareSerial.h>

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
class VEdirectReader {
    public:
        VEdirectReader(Connection *conn, String mqttTopic, u_int8_t RXpin, u_int8_t TXpin);
        void begin();
        void end();
        void tick();
        // HardwareSerial serialVE;
        // SoftwareSerial serialVE;
        EspSoftwareSerial::UART serialVE;
        void parse_message();

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
        // bool _match_sequence(uint16_t);
};

#endif