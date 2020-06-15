/**
 * config.h
 * CyGarage v2.0
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
//#define ENABLE_TLS                            // Comment this line to disable TLS (MQTT-over-SSL) support.
// The following option is disabled by default for security reasons. If you need basic unencrypted HTTP
// functionality (particularly if you are using CyGarageMobile v1.0), then uncomment the following line.
// /**
//  * @deprecated Will be removed in a future version.
//  */
//#define ENABLE_WEBSERVER                      // Comment this line to disable the embedded web server.
#define CONFIG_FILE_PATH "/config.json"         // The config file path. Do not alter unless you are sure.
#define DEFAULT_SSID "your_ssid_here"           // Put the SSID of your WiFi here.
#define DEFAULT_PASSWORD "your_wifi_password"   // Put your WiFi password here.
#ifdef ENABLE_WEBSERVER
    /**
     * @deprecated Will be removed in a future version.
     */
    #define WEBSERVER_PORT 80                   // The built-in webserver port.
#endif
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
IPAddress ip(192, 168, 0, 200);                 // The default static host IP.
IPAddress gw(192, 168, 0, 1);                   // The default static gateway IP.
IPAddress sm(255, 255, 255, 0);                 // The default static subnet mask.
IPAddress dns(gw);                              // The default static DNS server IP (same as gateway for most residential setups)

#endif