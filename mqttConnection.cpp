// #include "secrets.h"
#include "mqttConnection.h"
#include "commands.h"

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "nvslogger.h"
#include "timer.h"

NvsLogger nvs2;

// #include <SPI.h>
// #include <Adafruit_ADS1X15.h>

Connection::Connection()
{
    _wifiOk = false;
    _mqttOk = false;
    pinMode(_wifiLed, OUTPUT);
    pinMode(_mqttLed, OUTPUT);
    
}

DynamicJsonDocument _jsonDoc(uint16_t size) 
{
    DynamicJsonDocument doc(size);
    return(doc);
}

void WiFiStationWifiReady(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_READY");
}

void WiFiStationWifiScanDone(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_SCAN_DONE");
}

void WiFiStationStaStart(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_STA_START");
}

void WiFiStationStaStop(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_STA_START");
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_STA_CONNECTED");
    nvs2.log("Wifi connected");
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_STA_DISCONNECTED");
    nvs2.log("Wifi disconnected");
}

void WiFiStationAuthmodeChange(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_AUTH_MODE_CHANGED");
}

void WiFiStationGotIp(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_STA_GOT_IP");
    nvs2.log("Got IP");
}

void WiFiStationGotIp6(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_STA_GOT_IP_6");
    nvs2.log("Got IP6");
}

void WiFiStationLostIp(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_STA_LOST_IP");
    nvs2.log("Lost IP");
}

void WiFiApStart(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_AP_START");
}

void WiFiApStop(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_AP_STOP");
}

void WiFiApStaConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_AP_STACONNECTED");
}

void WiFiApStaDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_AP_STADISCONNECTED");
}

void WiFiApStaIpasSigned(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED");
}

void WiFiApProbeEwqRecved(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED");
}

void WiFiApGotIp6(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_AP_GOT_IP6");
}

void WiFiFtmReport(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("ARDUINO_EVENT_WIFI_FTM_REPORT");
}

void merge(JsonObject dest, JsonObjectConst src)
{
   for (JsonPairConst kvp : src)
   {
     dest[kvp.key()] = kvp.value();
   }
}

void Connection::setWifiSsid(String ssid) {
    _ssid = ssid;
}
void Connection::setWifiPasswd(String passwd){
    _passwd = passwd;
}
void Connection::setMqttHost(String host){
    _host = host;
}
void Connection::setMqttPort(int port) {
    _port = port;
}
void Connection::setMqttPort(String port) {
    _port = port.toInt();
}
void Connection::setMqttClientName(String clientName) {
    _clientName = clientName;
}

// void setSslCa(String ca) {
//     _wifiClient.setCACert(ca);
// }

// void setSslCert(String cert) {
//     _wifiClient.setCertificate(cert);
// }

// void setSslKey(String key) {
//     _wifiClient.setCertificate(key);
// }

void Connection::setMqttMainTopic(String mainTopic) {
    _callbackTopic = mainTopic + "/command";        // topic for receiving commands
    _crTopic = mainTopic + "/commandResponse";      // topic for sending command responses and return data
    // _jsonTopic = mainTopic + "/telemetry/json";     // topic for sending data observations
    _debugTopic = mainTopic + "/debug";             // topic for sending debug messages
}

void Connection::setStatusLeds()
{
    if (_wifiOk) {
        digitalWrite(_wifiLed, HIGH);
        // debug("wifiLed ON");
    }
    else {
        digitalWrite(_wifiLed, LOW);
        // debug("wifiLed OFF");
    }

    if (_mqttOk) {
        digitalWrite(_mqttLed, HIGH);
        // debug("mqttLed ON");
    }
    else {
        digitalWrite(_mqttLed, LOW);
        // debug("mqttLed OFF");
    }
}

void Connection::printAllParams()
{
    Serial.print("SSID: ");
    Serial.println(_ssid);
    Serial.print("Passwd: ");
    Serial.println(_passwd);
    Serial.print("MQTT port ");
    Serial.println(_port);
    Serial.print("MQTT host: ");
    Serial.println(_host);
}

void Connection::sendStatusToDebug()
{
    debug("SSID: " + _ssid);
    debug("Passwd: " + _passwd);
    debug("MQTT host : " + _host + ":" + _port);
    debug("RSSI:" + String(WiFi.RSSI()));
    debug("IP: " + String(WiFi.localIP().toString()));
}

void Connection::otaEnable() {

    ArduinoOTA
        .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
        })
        .onEnd([]() {
        Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

    ArduinoOTA.begin();
}

void Connection::otaDisable() {
    ArduinoOTA.end();
}

void Connection::otaLoop() {
    ArduinoOTA.handle();
}

void Connection::setStatusInterval(Timer t) {
    Timer _statusIntervalTimer = t;
}

void Connection::connect(
    String wifi_ssid, 
    String wifi_passwd, 
    String mqttHost, 
    String mainTopic, 
    std::function<void(char*, uint8_t*, unsigned int)> callback,
    String sslRootCa,
    String sslCert,
    String sslKey,
    uint16_t mqttPort=1883, 
    String mqttClientName="MqttClient", 
    int wifiLedPin=4, 
    int mqttLedPin=5
    )
{
    // set private variables
    _ssid = wifi_ssid;
    _passwd = wifi_passwd;
    _host = mqttHost;
    _port = mqttPort;
    _clientName = mqttClientName;
    _mainTopic = mainTopic;
    _wifiLed = wifiLedPin;
    _mqttLed = mqttLedPin;
    _sslRootCa = sslRootCa;
    _sslCert = sslCert;
    _sslKey = sslKey;
    
    _mqttClient.setServer(_host.c_str(), _port );
    _mqttClient.setClient(_wifiClient);
    _mqttClient.setCallback(callback);
    // _wifiClient.setCACert(_sslRootCa.c_str());
    // _wifiClient.setCertificate(_sslCert.c_str());
    // _wifiClient.setPrivateKey(_sslKey.c_str());
    
    // set all topics
    Connection::setMqttMainTopic(_mainTopic);
    
    Connection::wifiMqttConnect();

    _statusIntervalTimer.set(1, "hour");
}

void Connection::wifiMqttConnect() {
    // initalization function for establishing wifi connection
    Serial.print("Connecting to ");
    Serial.println(_ssid);
    WiFi.mode(WIFI_STA);
    
    // setup Wifi events
    WiFi.onEvent(WiFiStationWifiReady, ARDUINO_EVENT_WIFI_READY);
    WiFi.onEvent(WiFiStationWifiScanDone, ARDUINO_EVENT_WIFI_SCAN_DONE);
    WiFi.onEvent(WiFiStationStaStart, ARDUINO_EVENT_WIFI_STA_START);
    WiFi.onEvent(WiFiStationStaStop, ARDUINO_EVENT_WIFI_STA_STOP);
    WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent(WiFiStationAuthmodeChange, ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE);
    WiFi.onEvent(WiFiStationGotIp, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(WiFiStationGotIp6, ARDUINO_EVENT_WIFI_STA_GOT_IP6);
    WiFi.onEvent(WiFiStationLostIp, ARDUINO_EVENT_WIFI_STA_LOST_IP);
    WiFi.onEvent(WiFiApStart, ARDUINO_EVENT_WIFI_AP_START);
    WiFi.onEvent(WiFiApStop, ARDUINO_EVENT_WIFI_AP_STOP);
    WiFi.onEvent(WiFiApStaConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.onEvent(WiFiApStaDisconnected, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
    WiFi.onEvent(WiFiApStaIpasSigned, ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED);
    WiFi.onEvent(WiFiApProbeEwqRecved, ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED);
    WiFi.onEvent(WiFiApGotIp6, ARDUINO_EVENT_WIFI_AP_GOT_IP6);
    WiFi.onEvent(WiFiFtmReport, ARDUINO_EVENT_WIFI_FTM_REPORT);


    WiFi.begin(_ssid.c_str(), _passwd.c_str());
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED) { 
        Serial.print(".");
        digitalWrite(_wifiLed, HIGH);
        delay(200);
        digitalWrite(_wifiLed, LOW);
        delay(800);
        tries++;
        if (tries > 1000) {
            Serial.println("Connection Failed! Rebooting...");
            delay(5000);
            ESP.restart();
        }
        }
    Serial.println();
    _wifiOk = true;
    setStatusLeds();



    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());

    _mqttClient.setBufferSize(4096); // overrides MQTT_MAX_PACKET_SIZE in PubSubClient.h
    _mqttClient.connect(_clientName.c_str() );
    _mqttClient.subscribe(_callbackTopic.c_str() );
    debug("Connected to broker as " + _clientName);
}

void Connection::subscribeMqttTopic(String topic)
{
    _mqttClient.subscribe(topic.c_str());
    Connection::debug("Subscribing to topic " + topic);
}

// Timer send_network_info_timer(1, "hour");

void Connection::maintain()
{
    _mqttClient.loop();

    // check mqtt connection
    if ( ! _mqttClient.state() == 0 ) {
        nvs2.log("MQTT disconnected");
        _mqttOk = false;
        setStatusLeds();
        Serial.println(String("MQTT connection failed with code " + String(_mqttClient.state() )));
        Serial.println("Reconnecting");
        Connection::wifiMqttConnect();
    }
    else
    {
        _mqttOk = true;
    }

    // check wifi connection
    if ( WiFi.status() != WL_CONNECTED ) {
        nvs2.log("Wifi not connected");
        _wifiOk = false;
        setStatusLeds();
        Serial.println("Wifi not connected");
        Serial.println("Reconnecting");
        Connection::wifiMqttConnect();
    }
    else
    {
        _wifiOk = true;
    }
    setStatusLeds();

    // if (_statusIntervalTimer.is_done() ) {
    // if (send_network_info_timer.is_done() ) {    
    //     _mqttClient.publish((_mainTopic + "/ip").c_str(), WiFi.localIP().toString().c_str());
    //     _mqttClient.publish((_mainTopic + "/mac").c_str(), WiFi.macAddress().c_str());
    // }
}

PubSubClient Connection::get_mqttClient()
{
    return(_mqttClient);
}

int Connection::publish(String topic, String message)
{
    if (_mqttClient.publish(topic.c_str(), message.c_str()) ) {
        _mqttOk = true;
        setStatusLeds();
        digitalWrite(_mqttLed, !digitalRead(_mqttLed));
        delay(20);
        digitalWrite(_mqttLed, !digitalRead(_mqttLed));
        delay(20);
        return(0);
    }
    else {
        // something went wrong with mqtt publishing
        _mqttOk = false;
        setStatusLeds();
        return(1);
    }
    
}

// void Connection::publishTelemetry(String message) {
//     Connection::publish(_jsonTopic, message);
// }

void Connection::publishCommandResponse(String message) {
    // publish command response
    Connection::publish(_crTopic, message);
    // publish command response on telemetry topic
    // Connection::publishTelemetry(message);
}

void Connection::debug(String message)
{
    _mqttClient.publish( _debugTopic.c_str(), message.c_str() );
    Serial.print("DEBUG:");
    Serial.println(message);
}

void Connection::debug(int value)
{
    String message = String(value);
    _mqttClient.publish( _debugTopic.c_str(), message.c_str() );
    Serial.print("DEBUG:");
    Serial.println(message);
}

void Connection::debug(unsigned long value)
{
    String message = String(value);
    _mqttClient.publish(_debugTopic.c_str(), message.c_str());
    Serial.print("DEBUG:");
    Serial.println(message);
}

void Connection::debug(float value, int decimalPlaces = 2)
{
    String message = String(value, decimalPlaces);
    _mqttClient.publish(_debugTopic.c_str(), message.c_str() );
    Serial.print("DEBUG:");
    Serial.println(message);
}

unsigned long Connection::getTimestamp()
{
    NTPClient timeClient(_ntpUDP);
    timeClient.begin();
    timeClient.update();
    unsigned long timestamp = timeClient.getEpochTime();
    //int milliseconds = timeClient.get_millis
    return(timestamp);
}

String Connection::getTimestampMillis()
{
    unsigned long seconds = Connection::getTimestamp();
    String timestamp = String(seconds) + "000";
    return(timestamp);
}
