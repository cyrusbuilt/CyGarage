/**
 * config.h
 * CyGarage v2.2
 * 
 * (c) 2019,2020, Cyrus Brunner
 * 
 * This file provides default configuration values. Some of these values can
 * be overridden using a configuration file called 'config.json' in the root
 * of the SPIFFS filesystem. If configuration file cannot be found, cannot be
 * loaded, or does not contain certain configuration values, then these values
 * will be used instead.
 */

#ifndef config_h
#define config_h

#include <IPAddress.h>

// Configuration
//#define CG_MODEL_2                            // Comment this line to revert to Model 1 (single door) firmware.
#define ENABLE_OTA                              // Comment this line to disable OTA updates.
#define ENABLE_MDNS                             // Comment this line to disable the MDNS.
#define CONFIG_FILE_PATH "/config.json"         // The config file path. Do not alter unless you are sure.
#define DEFAULT_SSID "your_ssid_here"           // Put the SSID of your WiFi here.
#define DEFAULT_PASSWORD "your_wifi_password"   // Put your WiFi password here.
#define CLOCK_TIMEZONE -4                       // The timezone this device is located in. (For example, EST when observing DST = GMT-4, when not = GMT-5)
#define SERIAL_BAUD 115200                      // The BAUD rate (speed) of the serial port (console).
#define CHECK_WIFI_INTERVAL 30000               // How often to check WiFi status (milliseconds).
#define CHECK_SENSORS_INTERVAL 3000             // How often to check sensors (milliseconds).
#define CLOCK_SYNC_INTERVAL 3600000             // How often to sync the local clock with NTP (milliseconds).
#define ACTIVATION_DURATION 2500                // How long the activation relay should be on.
#define DEVICE_NAME "CYGARAGE"                  // The device name.
#define CHECK_MQTT_INTERVAL 35000               // MQTT connectivity check interval (milliseconds).
#define MQTT_TOPIC_STATUS "cygarage/status"     // MQTT status channel to publish to.
#define MQTT_TOPIC_CONTROL "cygarage/control"   // MQTT control channel to subscribe to.
#define MQTT_BROKER "your_mqtt_broker_IP"       // MQTT broker hostname or IP.
#define MQTT_PORT 8883                          // MQTT port number.
#ifdef ENABLE_OTA
    #include <ArduinoOTA.h>
    #define OTA_HOST_PORT 8266                     // The OTA updater port.
    #define OTA_PASSWORD "your_OTA_password_here"  // The OTA updater password.
#endif
IPAddress defaultIp(192, 168, 0, 200);                 // The default static host IP.
IPAddress defaultGw(192, 168, 0, 1);                   // The default static gateway IP.
IPAddress defaultSm(255, 255, 255, 0);                 // The default static subnet mask.
IPAddress defaultDns(defaultGw);                              // The default static DNS server IP (same as gateway for most residential setups)

typedef struct {
    // Network stuff
    String hostname;
    String ssid;
    String password;
    IPAddress ip;
    IPAddress gw;
    IPAddress sm;
    IPAddress dns;
    bool useDhcp;

    uint8_t clockTimezone;

    // MQTT stuff
    String mqttTopicStatus;
    String mqttTopicControl;
    String mqttBroker;
    String mqttUsername;
    String mqttPassword;
    uint16_t mqttPort;

    // OTA stuff
    uint16_t otaPort;
    String otaPassword;
} config_t;

#endif