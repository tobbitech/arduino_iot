#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "iot_capability.h"
#include "mqttConnection.h"



OnOffSwitch::OnOffSwitch(Connection * conn_pointer, int pin, String name, String mqtt_topic, String on_value, String off_value)
{
    // _conn = new Connection();
    _conn_pointer = conn_pointer;
    _pin = pin;
    _name = name;
    _mqtt_topic = mqtt_topic;
    _on_value = on_value;
    _off_value = off_value;
}

void OnOffSwitch::begin() {
    pinMode(_pin, OUTPUT);
    (*_conn_pointer).subscribeMqttTopic(_mqtt_topic);
    (*_conn_pointer).debug("OnOffSwitch " + _name + " created on topic " + _mqtt_topic);
    (*_conn_pointer).maintain();
    _conn_pointer->debug("Using '->' to call member function from pointer to object");
    
}

String OnOffSwitch::getMqttTopic() {
    return(_mqtt_topic);
}

String OnOffSwitch::getName() {
    return(_name);
}

void OnOffSwitch::turnOn(bool updateOnOffTopic)
{
    digitalWrite(_pin, HIGH);
    if (updateOnOffTopic) { _conn_pointer->publish(_mqtt_topic, _on_value); }
    (*_conn_pointer).debug("OnOffSwitch " + _name + " turned ON");
    _conn_pointer->maintain();
}


void OnOffSwitch::turnOff(bool updateOnOffTopic)
{
    digitalWrite(_pin, LOW);
    if (updateOnOffTopic) { _conn_pointer->publish(_mqtt_topic, _off_value); }
    (*_conn_pointer).debug("OnOffSwitch " + _name + " turned OFF");
    _conn_pointer->maintain();
}

void OnOffSwitch::toggle(bool updateOnOffTopic)
{
    int state = digitalRead(_pin);
    if (state == HIGH) {
        OnOffSwitch::turnOn(updateOnOffTopic);
    }
    else {
        OnOffSwitch::turnOff(updateOnOffTopic);
    }
}

void OnOffSwitch::setSwitchState(String on_off_value, bool updateOnOffTopic)
{
    if (on_off_value == _on_value) {
        OnOffSwitch::turnOn(updateOnOffTopic);
    }
    else if (on_off_value == _off_value)
    {
        OnOffSwitch::turnOff(updateOnOffTopic);
    }
    else
    {
        // a unexpected value was received. Log and do nothing.
        (*_conn_pointer).debug("Unexpected switch state received: " + on_off_value);
    }
}

String OnOffSwitch::getOnValue()
{
    return(_on_value);
}

String OnOffSwitch::getOffValue()
{
    return(_off_value);
}



DS18B20_temperature_sensors::DS18B20_temperature_sensors(Connection * conn_pointer, int pin, String mqtt_main_topic): _oneWire(pin), _sensors(&_oneWire) 
// dunno why this works, but initating DallasTemperature objects in contructor does not work
{
    _conn_pointer = conn_pointer;
    _mqtt_main_topic = mqtt_main_topic;
    _numberOfDevices = 0;
    _mapSize = 0;
}

uint8_t DS18B20_temperature_sensors::scanForSensors()
{
    // returns the number of sensors found on the bus
    _conn_pointer->debug("Scanning for DS18B20 sensors");
    _sensors.begin();
    _numberOfDevices = _sensors.getDeviceCount();
    _conn_pointer->debug("Found " + String(_numberOfDevices) + " DS18B20 sensors");

    // reset address and name arrays
    for (int i = 0; i < _numberOfDevices; i++) {
        _deviceNames[i] = "";
        for (byte j = 0; j < 8; j++) {
            _deviceAddresses[i][j] = 0;
        }
    }

    // store address for all devices in _deviceAddresses
    for (int i = 0; i < _numberOfDevices; i++) {
        _sensors.getAddress(_deviceAddresses[i], i);
        if ( _deviceNames[i].length() == 0) {
            _deviceNames[i] = DS18B20_temperature_sensors::getAddressString(i);
            _conn_pointer->debug("Devicename: " + _deviceNames[i]);
            for( int j = 0; j < _mapSize; j++) {
                if ( _deviceNames[i] == _addressMap[j]) {
                    _deviceNames[i] = _nameMap[j];
                    _conn_pointer->debug("Mapped name " + _nameMap[j] + " to device address " + getAddressString(i));
                }
            }
        }
    }


    // _sensors.requestTemperatures();
    return(_numberOfDevices);
}

String DS18B20_temperature_sensors::convertAddressToString(DeviceAddress address)
{
    // Converts address of type DeviceAddress to String
    String deviceName = "";
    for (byte i = 0; i < 8; i++) {
        address[i];
        deviceName += String(address[i], HEX);
        if (i < 7) {
            deviceName += ":";
        }
    }
    return(deviceName);
}

String DS18B20_temperature_sensors::getAddressString(int deviceIndex){
    // Converts address of type DeviceAddress to String
    return(DS18B20_temperature_sensors::convertAddressToString(_deviceAddresses[deviceIndex]));
}

void DS18B20_temperature_sensors::mapNameToDeviceAddress(DeviceAddress address, String name)
{
    _addressMap[_mapSize] = convertAddressToString(address);
    _nameMap[_mapSize] = name;
    _mapSize++;
}

String DS18B20_temperature_sensors::getMqttTopic() {
    return(_mqtt_main_topic);
}

String DS18B20_temperature_sensors::getName(int deviceIndex) {
    return(_deviceNames[deviceIndex]);
}

void DS18B20_temperature_sensors::publishAllTemperatures()
{
    _sensors.requestTemperatures();
    for (int i = 0; i < _numberOfDevices; i++) {
        String deviceMqttTopic = _mqtt_main_topic + "/" + _deviceNames[i];
        _conn_pointer->publish(deviceMqttTopic, String(_sensors.getTempC(_deviceAddresses[i])));
        _conn_pointer->debug(_deviceNames[i] + ": " + _sensors.getTempC(_deviceAddresses[i]) + "C");
    }
}   


InputMomentary::InputMomentary(
            Connection * conn_pointer, 
            int pin, 
            String name, 
            String mqtt_topic, 
            uint8_t mode,
            String on_value, 
            String off_value
    ) {
        _conn_pointer = conn_pointer;
        _pin = pin;
        _name = name;
        _mqtt_topic = mqtt_topic;
        _on_value = on_value;
        _off_value = off_value;
        _mode = mode;
        _threshold_voltage = 3.3/2;
        _debounce_ms = 500;

    }

void InputMomentary::begin() {
    // pinMode must be set elsewhere

    // sets all switches to false on boot,
    // if not "true" state will linger
    _conn_pointer->publish(_mqtt_topic, false);
}

void InputMomentary::check() {
    // to be run as often as possible
    int16_t value;

    if (_debounce_timer.is_done() ) {    
        bool state = false;
        if (_mode == INPUT_MOMENTARY_ANALOG) {
            value = analogRead(_pin);
            uint16_t threshold = round(4096 / 3.3) * _threshold_voltage;
            if (value > threshold) {
                state = true;
            }
        } else {
            value = digitalRead(_pin);
            if (_mode == INPUT_MOMENTARY_HIGH_ON) {
                if (value > 0) {
                    state = true;
                }
            }
            if (_mode == INPUT_MOMENTARY_LOW_ON) {
                if (value == 0) {
                    state = true;
                }
            } 
        }

        if (state == true ) { _debounce_timer.set(_debounce_ms, "milliseconds"); }

        if (state != _last_state) {
            _last_state = state;
            String debug_text = "Momentary input " + _name + " changed to: " + String(state);
            if( _mode == INPUT_MOMENTARY_ANALOG) { debug_text += " with value " + String(value); }
            _conn_pointer->debug(debug_text);
            String value_str = _on_value;
            if (state == false) { value_str = _off_value;}
            _conn_pointer->publish(_mqtt_topic, value_str );
        }
    }

}


void InputMomentary::set_threshold_voltage(float new_threshold_voltage) {
    _threshold_voltage = new_threshold_voltage;
}