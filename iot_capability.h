#ifndef IOT_CAPABILITY_H
#define IOT_CAPABILITY_H

#include <Arduino.h>
#include "mqttConnection.h"

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

#endif