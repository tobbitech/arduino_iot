#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "iot_capability.h"
#include "mqttConnection.h"
#include <HardwareSerial.h>


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

    }

void InputMomentary::begin() {
    // pinMode must be set elsewhere
    

}

void InputMomentary::check() {
    // to be run as often as possible
    int16_t value;
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

    if (state != _last_state) {
        _last_state = state;
        String debug_text = "Momentary input " + _name + " changed to: " + String(state);
        if( _mode == INPUT_MOMENTARY_ANALOG) { debug_text += " with value " + String(value); }
        _conn_pointer->debug(debug_text);
        String value = _on_value;
        if (state == false) { value = _off_value;}
        _conn_pointer->publish(_mqtt_topic, value);
    }

}


void InputMomentary::set_threshold_voltage(float new_threshold_voltage) {
    _threshold_voltage = new_threshold_voltage;
}

DebounceButton::DebounceButton(
            Connection * conn_pointer, 
            int pin, 
            String name, 
            String mqtt_topic,
            float analog_threshold_V = 0,
            bool on_level,
            u_int32_t debounce_delay,
            String on_value, 
            String off_value)
{
    _conn_pointer = conn_pointer;
    _pin = pin;
    _mqtt_topic = mqtt_topic;
    _name = name;
    _analog_threshold_V = analog_threshold_V;
    _pressed = on_level;
    _unpressed = !on_level;
    _debounce_delay = debounce_delay;
    _on_value = on_value;
    _off_value = off_value;

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


bool DebounceButton::is_pressed() {
    return _is_pressed;
}

bool DebounceButton::is_released() {
    return _is_released;
}

bool DebounceButton::is_held() {
    return _is_held;
}

bool DebounceButton::is_sticky_held() {
    return (_is_sticky_held);
}

u_int32_t DebounceButton::get_hold_time_ms() {
    return millis() -_hold_time_ms;
}

void DebounceButton::tick() {
    switch_value = _unpressed;
    if (_analog_threshold_V == 0 ) {
        switch_value = digitalRead(_pin);
    }
    else {
        uint16_t value = analogRead(_pin);
        uint16_t threshold = round(4096 / 3.3) * _analog_threshold_V;
        if (value > threshold) {
            state = _pressed;
        }
    }
    _last_state = _state;

    switch(_state) {
        case DebounceButton::RESET:
            _is_pressed = false;
            _is_held = false;
            _is_released = false;
            _hold_time_ms = millis();
            _state = DebounceButton::START;
            break;
        case DebounceButton::START:
            if (switch_value == _pressed) {
                _state = DebounceButton::GO;
            } 
            break;
        case DebounceButton::GO:
            _debounce_timer.set(_debounce_delay, "milliseconds");
            _state = DebounceButton::WAIT;
            break;
        case DebounceButton::WAIT:
            if (switch_value == _unpressed) {
                _state = DebounceButton::RESET;
            } else if (_debounce_timer.is_done()) {
                _state = DebounceButton::TRIGGERED;
            }
            break;
        case DebounceButton::TRIGGERED:
            _is_pressed = true;
            _hold_time_ms = millis();
            // Serial.println("Button pressed");
            _sticky_timer.reset();
            _conn_pointer->publish(_mqtt_topic, _on_value);
            _state = DebounceButton::HELD;
            break;

        case DebounceButton::HELD:
            _is_pressed = false;
            _is_held = true;
            _is_sticky_held = true;
            if (switch_value == _unpressed ) {
                _state = DebounceButton::STICKY;
            }
            break;
        case DebounceButton::STICKY:
            _is_held = false;
            if (_sticky_timer.is_done() ) {
                _state = DebounceButton::RELEASED;
            }
            break;
        case DebounceButton::RELEASED:
            _is_sticky_held = false;
            _is_released = true;
            _conn_pointer->publish(_mqtt_topic, _off_value);
            _state = DebounceButton::RESET;
            break;
    }

    //debug
    if (_state != _last_state) {
        _conn_pointer->debug("Button " + _name + " changed state to: " + String(_state));
    }
}




HANreader::HANreader(Connection * conn, String mqttTopic, uint8_t RXpin, uint8_t TXpin): serialHAN(1)
{
    _RXpin = RXpin;
    _TXpin = TXpin;
    _conn = conn;
    _mqttTopic = mqttTopic;
}

void HANreader::begin() {
    serialHAN.begin(2400, SERIAL_8N1, _RXpin, _TXpin);
    _last_byte_millis = 0;
    _message = "";
    _message_buf_pos = 0;
}

void HANreader::end() {
    serialHAN.end();
}

void HANreader::tick() {
    uint32_t time_since_last_byte = millis() - _last_byte_millis;

    if ( time_since_last_byte > HAN_READ_TIMEOUT_MS && _message != "" ) {
        // parse_message( _message );
        _message_buf[_message_buf_pos] = '\0';
        parse_message();
        _message = "";
        _message_buf_pos = 0;
    }

    if ( serialHAN.available() > 0 ) {
        char recv_char = serialHAN.read();
        _last_byte_millis = millis(); // reset timeout counter
        // if (recv_char != NULL) {
        _message += recv_char;
        _message_buf[_message_buf_pos++] = recv_char;
            

        // }
    }
}

u_int16_t crc16x25(unsigned char *data_p, u_int16_t lenght) {
    // calculates CRC16/X25
    u_int16_t crc = 0xFFFF;
    u_int32_t data;
    u_int16_t crc16_table[] = {
            0x0000, 0x1081, 0x2102, 0x3183,
            0x4204, 0x5285, 0x6306, 0x7387,
            0x8408, 0x9489, 0xa50a, 0xb58b,
            0xc60c, 0xd68d, 0xe70e, 0xf78f
    };

    while(lenght--){
        crc = ( crc >> 4 ) ^ crc16_table[(crc & 0xf) ^ (*data_p & 0xf)];
        crc = ( crc >> 4 ) ^ crc16_table[(crc & 0xf) ^ (*data_p++ >> 4)];
    }

    data = crc;
    return (~crc);
}

void HANreader::parse_message() {
    // translate char string to hex
    String hex = "";
    for (int i = 0; i < _message_buf_pos; i++ ) {
            char buf[5];
            sprintf(buf, "%02x", _message_buf[i] );
            hex += String(buf);  
    }
    // publish raw HAN message
    _conn->publish(_mqttTopic + "/hex", hex + " len: " + String(_message_buf_pos));
    // _conn->publish(_mqttTopic + "/raw", _message_buf);

    // define all OBIS codes
    han_line version {.obis_code = { 0x01, 0x01, 0x00, 0x02, 0x81, 0xff }, .name="OBIS list version", .unit="", .topic="obis_list_version"  };
    han_line id {.obis_code = { 0x00, 0x00, 0x60, 0x01, 0x00, 0xff }, .name="Meter ID", .unit="", .topic="meter_id"  };
    han_line type {.obis_code = { 0x00, 0x00, 0x60, 0x01, 0x07, 0xff }, .name="Meter type", .unit="", .topic="meter_type"  };
    
    han_line active_import {.obis_code = { 0x01, 0x00, 0x01, 0x07, 0x00, 0xff }, .name="Active import", .unit="W", .topic="active_import_W"  };
    han_line active_export {.obis_code = { 0x01, 0x00, 0x02, 0x07, 0x00, 0xff }, .name="Active export", .unit="W", .topic="active_export_W"  };
    han_line reactive_import {.obis_code = { 0x01, 0x00, 0x03, 0x07, 0x00, 0xff }, .name="Reactive import", .unit="VAr", .topic="reactive_import_VAr"  };
    han_line reactive_export {.obis_code = { 0x01, 0x00, 0x04, 0x07, 0x00, 0xff }, .name="Reactive export", .unit="VAr", .topic="reactive_export_VAr"  };

    han_line current_L1 {.obis_code = { 0x01, 0x00, 0x1f, 0x07, 0x00, 0xff }, .name="Current L1", .unit="A", .topic="current_l1_A"  };
    han_line current_L2 {.obis_code = { 0x01, 0x00, 0x33, 0x07, 0x00, 0xff }, .name="Current L2", .unit="A", .topic="current_l2_A"  };
    han_line current_L3 {.obis_code = { 0x01, 0x00, 0x47, 0x07, 0x00, 0xff }, .name="Current L3", .unit="A", .topic="current_l3_A"  };

    han_line voltage_L1 {.obis_code = { 0x01, 0x00, 0x20, 0x07, 0x00, 0xff }, .name="Voltage L1", .unit="V", .topic="voltage_l1_V"  };
    han_line voltage_L2 {.obis_code = { 0x01, 0x00, 0x34, 0x07, 0x00, 0xff }, .name="Voltage L2", .unit="V", .topic="voltage_l2_V"  };
    han_line voltage_L3 {.obis_code = { 0x01, 0x00, 0x48, 0x07, 0x00, 0xff }, .name="Voltage L3", .unit="V", .topic="voltage_l3_V"  };

    han_line meter_clock {.obis_code = { 0x00, 0x00, 0x01, 0x00, 0x00, 0xff }, .name="Clock", .unit="", .topic="clock"  };
   
    han_line cum_active_import {.obis_code =   { 0x01, 0x00, 0x01, 0x08, 0x00, 0xff }, .name="Cummulative active import", .unit="kWh", .topic="cum_active_import_kWh"  };
    han_line cum_active_export {.obis_code =   { 0x01, 0x00, 0x02, 0x08, 0x00, 0xff }, .name="Cummulative active export", .unit="kWh", .topic="cum_active_export_kWh"  };
    han_line cum_reactive_import {.obis_code = { 0x01, 0x00, 0x03, 0x08, 0x00, 0xff }, .name="Cummulative reactive import", .unit="kVArh", .topic="cum_reactive_import_kVArh"  };
    han_line cum_reactive_export {.obis_code = { 0x01, 0x00, 0x04, 0x08, 0x00, 0xff }, .name="Cummulative reactive export", .unit="kVArh", .topic="cum_reactive_export_kVArh"  };

    han_line han_lines[] = {version, id, type, active_import, active_export, reactive_import, reactive_export, current_L1, current_L2, current_L3, 
                            voltage_L1, voltage_L2, voltage_L3, meter_clock, cum_active_import, cum_active_export, cum_reactive_import, cum_reactive_export };
    _no_han_lines = 18;

    // parse HAN message
    int i = 0;
    if (_message_buf[i++] == 0x7e ) { 
        // flag found
        u_int8_t header[6] = {
            _message_buf[i++],
            _message_buf[i++],
            _message_buf[i++],
            _message_buf[i++],
            _message_buf[i++],
            _message_buf[i++]
        };
        u_int16_t header_checksum = _message_buf[i++] | _message_buf[i++] << 8;
        u_int16_t calc_header_checksum = crc16x25(header, 6);
        if (header_checksum == calc_header_checksum ) {
            // _conn->debug("Header checksum OK! " + String(header_checksum, HEX) );
        } else {
            _conn->debug("Header checksum error. Dropping package. Package: " + String(header_checksum, HEX)  + " calc: " + String(calc_header_checksum) );
            return;
        }
        
        // jump past next 9 bytes, we dont need them for anything
        i = i + 9; 

        int datatype = _message_buf[i++];
        int payload_lines = _message_buf[i++];

        String name = "";
        String unit = "";
        String subtopic = "";
        String value_str = "";

        for (int line = 0; line < payload_lines; line++) {
            i += 4; // jump past type identifier in line
            u_int8_t obis_code[6] = {
                _message_buf[i++],
                _message_buf[i++],
                _message_buf[i++],
                _message_buf[i++],
                _message_buf[i++],
                _message_buf[i++]
            };

            // printf("OBIS code: %02x %02x %02x %02x %02x %02x\t", obis_code[0], obis_code[1], obis_code[2], obis_code[3], obis_code[4], obis_code[5] );


            for (int l = 0; l < _no_han_lines; l++) {
                if (std::equal(obis_code, obis_code + sizeof obis_code / sizeof *obis_code, han_lines[l].obis_code) ) {
                    name = han_lines[l].name;
                    unit = han_lines[l].unit;
                    subtopic = han_lines[l].topic;
                    // _conn->debug("OBIS code found: " + name + ", subtopic: " + subtopic );
                    break;
                } else {
                    // _conn->debug("No OBIS code found");
                }
             }

            u_int8_t variable_type = _message_buf[i++];

            if (variable_type == 0x0a ) {
                // this is a string, have to find length
                int string_length = _message_buf[i++];
                char string_contents[string_length+1];
                int j;
                for (j=0; j<string_length;j++) {
                     string_contents[j] = _message_buf[i++];
                }
                string_contents[j] = '\0';
                value_str = string_contents;
            } else if (variable_type == 0x06) {
                // this is a uint32 -> Energy, cumulative energy
                u_int32_t value;
                value = _message_buf[i+3] | _message_buf[i+2] << 8 | _message_buf[i+1] << 16 | _message_buf[i] << 24;
                i += 4 + 6; // +6 is the stuff after the value on each line
                value_str = String(value);

                if (name == "Cummulative active import" ||
                    name == "Cummulative active export" ||
                    name == "Cummulative reactive import" ||
                    name == "Cummulative reactive export" ) {
                        float value_f = value / 100.0; // cumulative energy has resolution 0.01kWh
                        value_str = String(value_f, 2);
                }
            } else if (variable_type == 0x9) {
                // this is clock time - octet-string
                int string_length = _message_buf[i++];
                value_str = "";
                uint16_t year  = _message_buf[i+1] | _message_buf[i] << 8;
                i += 2;
                uint8_t  month = _message_buf[i++];
                uint8_t  day   = _message_buf[i++];
                uint8_t  dow   = _message_buf[i++];
                uint8_t  hour  = _message_buf[i++];
                uint8_t  minute= _message_buf[i++];
                uint8_t  second= _message_buf[i++];

                value_str = String(year) + "." + String(month) + "." + String(day) + "-"
                        + String(hour) + ":" + String(minute) + ":" + String(second) + " ";

                for (int j = 8; j < string_length; j++) {
                    uint8_t octet = _message_buf[i++];
                    value_str += String(octet);
                    if ( j < string_length -1 ) { 
                        value_str += ".";
                    }
                }            
            } else if ( variable_type == 0x10 ){
                // this is a i16 -> Current
                int16_t value;
                value = _message_buf[i+1] | _message_buf[i] << 8;
                i += 2 + 6; // +6 is the stuff after the value on each line
                value_str = String(value);
                if (name=="Current L1" || name=="Current L2" || name=="Current L3" ) {
                    float value_f = value / 10.0; // Current has 0.1A resolution
                    value_str = String(value_f, 1);
                }
            } else if ( variable_type == 0x12 ) {
                // this is a u16 -> Voltage
                uint16_t value;
                value = _message_buf[i+1] | _message_buf[i] << 8;
                i += 2 + 6; // +6 is the stuff after the value on each line
                if (name=="Voltage L1" || name=="Voltage L2" || name=="Voltage L3" ) {
                    float value_f = value / 10.0; // Voltage has 0.1A resolution
                    value_str = String(value_f, 1);
                }
            }
            
            _conn->publish(_mqttTopic + "/" + subtopic, value_str );
        }

        // checking packet checksum
        u_int16_t packet_checksum = _message_buf[i++] | _message_buf[i++] << 8;
        u_int16_t calc_packet_checksum = crc16x25(_message_buf + 1, i-3);
        if (packet_checksum == calc_packet_checksum ) {
            // _conn->debug("Packet checksum OK! " + String(packet_checksum, HEX) );
        } else {
            _conn->debug("Packet checksum error. Dropping package. Package: " + String(packet_checksum, HEX)  + " calc: " + String(calc_packet_checksum) );
            return;
        }
        
        if (_message_buf[i] != 0x7e) {
            _conn->debug("No end flag found. Instead found: " + String(_message_buf[i], HEX) + " Dropping package.");
        }
    }
}


VEdirectReader::VEdirectReader(Connection * conn, String mqttTopic, uint8_t RXpin, uint8_t TXpin): serialVE(2)
{
    _RXpin = RXpin;
    _TXpin = TXpin;
    _conn = conn;
    _mqttTopic = mqttTopic;
}

void VEdirectReader::begin() {
    serialVE.begin(19200, SERIAL_8N1, _RXpin, _TXpin); // for hardwareserial
    _send_raw_data_timer.set(100, "seconds");
    set_publish_timer_s(5);
    _last_byte_millis = 0;
    _message = "";
    _message_buf_pos = 0;
    _voltage_V = 0;
    _current_A = 0;
    _power_W = 0;
    _soc = 0;
    _soc_by_v = 0;
    _pv_voltage_V = 0;
    _pv_power_W = 0;
    _voltage_is_set = false;
    _current_is_set = false;
    _power_is_set = false;
    _soc_is_set = false;
    _pv_voltage_is_set = false;
    _pv_power_is_set = false;
    _yield_total_is_set = false;
    _yield_today_is_set = false;
    _max_power_today_is_set = false;
    _yield_yesterday_is_set = false;
    _max_power_yesterday_is_set = false;
    
}

void VEdirectReader::end() {
    serialVE.end();
}

void VEdirectReader::tick() {
    uint32_t time_since_last_byte = millis() - _last_byte_millis;

    if ( time_since_last_byte > VEDIRECT_TIMEOUT_MS && _message != "" ) {
        _message_buf[_message_buf_pos] = '\0';
        parse_message();
        _message = "";
        _message_buf_pos = 0;
    }

    if ( serialVE.available() > 0 ) {
        char recv_char = serialVE.read();
        _last_byte_millis = millis(); // reset timeout counter
        _message += recv_char;
        _message_buf[_message_buf_pos++] = recv_char;
    }
}

void VEdirectReader::set_publish_timer_s(u_int16_t seconds) {
    _publish_data_timer.set(seconds, "seconds");
}

void VEdirectReader::publish_data() {
    if (_voltage_is_set) {
        _conn->publish(_mqttTopic + "/battery_voltage_V", String(_voltage_V, 2));
        _conn->publish(_mqttTopic + "/soc_by_v", String(_soc_by_v, 1));
    }
    if (_current_is_set) {
        _conn->publish(_mqttTopic + "/current_I", String(_current_A, 2));
    }
    if (_power_is_set) {
        _conn->publish(_mqttTopic + "/power_W", String(_power_W, 0));
    }
    if (_soc_is_set) {
        _conn->publish(_mqttTopic + "/soc_%", String(_soc, 1));
    }
    if (_pv_voltage_is_set) {
        _conn->publish(_mqttTopic + "/pv_voltage_V", String(_pv_voltage_V, 2));
    }
    if (_pv_power_is_set) {
        _conn->publish(_mqttTopic + "/pv_power_W", String(_pv_power_W, 0));
    }
    if (_yield_total_is_set) {
        _conn->publish(_mqttTopic + "/yield_total_kWh", String(_yield_total_kWh, 2));
    }
    if (_yield_today_is_set) {
        _conn->publish(_mqttTopic + "/yield_today_kWh", String(_yield_today_kWh, 2));
    }
    if (_max_power_today_is_set) {
        _conn->publish(_mqttTopic + "/max_power_today_W", String(_max_power_today_W, 0));
    }
    if (_yield_yesterday_is_set) {
        _conn->publish(_mqttTopic + "/yield_yesterday_kWh", String(_yield_yesterday_kWh, 2));
    }
    if (_max_power_yesterday_is_set) {
        _conn->publish(_mqttTopic + "/max_power_yesterday_W", String(_max_power_yesterday_W, 0));
    }
}

void VEdirectReader::parse_message() {
    if( _send_raw_data_timer.is_done() ) {
        _conn->publish(_mqttTopic + "/VEdirect/raw", _message);
    }

    String key = "none";
    String value ="empty";

    bool separator_found = false;

    for (int i = 0; i < _message_buf_pos; i++) {
        if ( _message[i] == '\n') {
            if (key == "V") {
                _voltage_V = value.toInt() / 1000.0;
                _soc_by_v = (0.9369*_voltage_V*_voltage_V - 87.69*_voltage_V + 2050);
                if (_soc_by_v > 100 ) { _soc_by_v = 100; }
                _voltage_is_set = true;
                //trollslottetBatterySOCbyV.sendCommand((0.009369*Math.pow(x,2) - 0.8769*x + 20.5)*100);
//88.69*Math.pow(x,6) - 151.5*Math.pow(x,5) - 21.37*Math.pow(x,4) + 179.9*Math.pow(x,3) - 125.7*Math.pow(x,2) + 39.33*x + 48);
            }
            else if (key == "I") {
                _current_A = value.toInt() / 1000.0;
                _current_is_set = true;
            }
            else if (key == "P") {
                _power_W = value.toInt();
                _power_is_set = true;
            }
            else if (key == "SOC") {
                _soc = value.toInt() / 10;
                _soc_is_set = true;
            }
            else if (key == "VPV") {
                _pv_voltage_V = value.toInt() / 1000.0;
                _pv_voltage_is_set = true;
            }
            else if (key == "PPV") {
                _pv_power_W = value.toInt();
                _pv_power_is_set = true;
            }
            else if (key == "H19") {
                _yield_total_kWh = value.toInt() / 100.0;
                _yield_total_is_set = true;
            }
            else if (key == "H20") {
                _yield_today_kWh = value.toInt() / 100.0;
                _yield_today_is_set = true;
            }
            else if (key == "H21") {
                _max_power_today_W = value.toInt();
                _max_power_today_is_set = true;
            }
            else if (key == "H22") {
                _yield_yesterday_kWh = value.toInt() / 100.0;
                _yield_yesterday_is_set = true;
            }
            else if (key == "H23") {
                _max_power_yesterday_W = value.toInt();
                _max_power_yesterday_is_set = true;
            }

            key = "";
            value = "";
            separator_found = false;
        }
        else if (_message[i] == '\t') {
            // separator found!
            separator_found = true;
        }
        else if (separator_found == false ) {
            // get data field:
            key += _message[i];
        }
        else {
            value += _message[i];
        }
    }
}