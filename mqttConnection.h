#ifndef MQTT_CONNECTION_H
#define MQTT_CONNECTION_H

#include "commands.h"
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include "timer.h"
#include "return_codes.h"

// #define ARDUINO_IOT_USE_SSL

// all library code before endif


class Connection 
{
    public:
        Connection();
        void connect(            String wifi_ssid, 
            String wifi_passwd, 
            String mqttHost, 
            String mainTopic, 
            std::function<void(char*, uint8_t*, unsigned int)> callback,
            String sslRootCa,
            String sslCert,
            String sslKey,
            uint16_t mqttPort, 
            String mqttClientName, 
            int wifiLedPin, 
            int mqttLedPin,
            bool useSSL
        );
        void wifiMqttConnect();
        void maintain();
        PubSubClient get_mqttClient();
        void printAllParams();
        void sendStatusToDebug();
        void debug(String message);
        void debug(int message);
        void debug(unsigned long message);
        void debug(float message, int decimalPlaces);
        int publish(String topic, String message);
        void publishTelemetry(String message);
        void publishCommandResponse(String message);
        void setStatusLeds();
        unsigned long getTimestamp();
        String getTimestampMillis();
        void setWifiSsid(String ssid);
        void setWifiPasswd(String passwd);
        void setMqttHost(String host);
        void setMqttPort(int port);
        void setMqttPort(String port);
        void setMqttClientName(String clientName);
        void setMqttMainTopic(String mainTopic);
        void subscribeMqttTopic(String topic);
        void setSslCa(String ca);
        void setSslCert(String cert);
        void setSslKey(String key);
        void otaEnable();
        void otaDisable();
        void otaLoop();
        void setStatusInterval(Timer t);

    private:
        void(*commandFunctionPointer)(String, int, JsonArray, bool);
        String _ssid;
        String _passwd;
        String _host;
        bool _useSSL;
        uint16_t _port;
        String _clientName;
        String _mainTopic;

        // #ifdef ARDUINO_IOT_USE_SSL
        // WiFiClientSecure _wifiClient = WiFiClientSecure();
        // #else
        WiFiClient _wifiClient = WiFiClient();
        WiFiClientSecure _wifiSecureClient = WiFiClientSecure();
        // #endif

        PubSubClient _mqttClient;
        String _callbackTopic;
        String _debugTopic;
        String _jsonTopic;
        String _crTopic;
        String _sslRootCa;
        String _sslKey;
        String _sslCert;
        bool _mqttOk;
        bool _wifiOk;
        int _wifiLed;
        int _mqttLed;
        WiFiUDP _ntpUDP;
        NTPClient _timeClient(WiFiUDP);
        Timer _statusIntervalTimer;
};

#endif