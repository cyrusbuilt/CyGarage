/**
 * @file main.cpp
 * @author Chris "Cyrus" Brunner (cyrusbuilt at gmail dot com)
 * @brief  This is the firmware for CyGarage, an IoT garage door opener add-on. The device is intended
 * to integrate with an existing garage door opener and connect via WiFi, providing the ability
 * to sense when the door is open/closed/ajar and provide a means of opening/closing the door.
 * This firmware also allows for OTA firmware updates, and additionally can be integrated with
 * a home automation server such as OpenHAB.
 * @version 2.3
 * @date 2022-11-15
 * 
 * @copyright Copyright (c) 2022 Cyrus Brunner
 */

#ifndef ESP8266
    #error This firmware is only compatible with ESP8266 controllers.
#endif
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <FS.h>
#include <time.h>
#include <TZ.h>
#include "DoorContact.h"
#include "LED.h"
#include "Relay.h"
#include "TaskScheduler.h"
#include "ResetManager.h"
#include "ESPCrashMonitor.h"
#include "ArduinoJson.h"
#include "PubSubClient.h"
#include "config.h"
#include "TelemetryHelper.h"
#include "Console.h"

#define FIRMWARE_VERSION "2.3"

// Startup delay before processing MQTT messages.
#define MQTT_STARTUP_DELAY 5000

// Pin definitions
#define PIN_SENSOR_LED 5
#define PIN_WIFI_LED 2
#define PIN_ACTIVATE 4
#define PIN_OPEN_SENSOR 12
#define PIN_CLOSE_SENSOR 13
#ifdef CG_MODEL_2
    #define PIN_OPEN_SENSOR_2 14
    #define PIN_CLOSE_SENSOR_2 16
    #define PIN_ACTIVATE_2 15
#endif

// Forward declarations
void onDoorContactStateChange(DoorInfo* sender);
void onCheckWiFi();
void onCheckActivation();
void failSafe();
void onCheckMqtt();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
void onSyncClock();
void onEnableMqtt();

// Global vars
#ifdef ENABLE_MDNS
    #include <ESP8266mDNS.h>
    MDNSResponder mdns;
#endif
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
DoorContact openSensor(PIN_OPEN_SENSOR, onDoorContactStateChange);
DoorContact closeSensor(PIN_CLOSE_SENSOR, onDoorContactStateChange);
Relay garageDoorRelay(PIN_ACTIVATE, NULL, "GARAGE_DOOR");
#ifdef CG_MODEL_2
    DoorContact openSensor2(PIN_OPEN_SENSOR_2, NULL);
    DoorContact closeSensor2(PIN_CLOSE_SENSOR_2, NULL);
    Relay garageDoorRelay2(PIN_ACTIVATE_2, NULL, "GARAGE_DOOR_2");
#endif
LED sensorLED(PIN_SENSOR_LED, NULL);
LED wifiLED(PIN_WIFI_LED, NULL);
Task tCheckWifi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWiFi);
Task tCheckActivation(ACTIVATION_DURATION, TASK_FOREVER, &onCheckActivation);
Task tCheckMqtt(CHECK_MQTT_INTERVAL, TASK_FOREVER, &onCheckMqtt);
Task tSyncClock(CLOCK_SYNC_INTERVAL, TASK_FOREVER, &onSyncClock);
Task tEnableMqtt(MQTT_STARTUP_DELAY, TASK_ONCE, &onEnableMqtt);
Scheduler taskMan;
config_t config;
String lastState = "";
#ifdef CG_MODEL_2
    String lastState2 = "";
#endif
bool filesystemMounted = false;
volatile SystemState sysState = SystemState::BOOTING;
volatile bool ignoreMqtt = true;

/**
 * @brief Read the specified sensor pins and determine the door state. Prints
 * the result to console and returns the state as string.
 * @return The state string.
 */
String deviceStatus() {
    String state = "AJAR";
    if (openSensor.isOpen()) {
        state = "OPEN";
    }
    else if (closeSensor.isOpen()) {
        state = "CLOSED";
    }

    Serial.print(F("INFO: Door status = "));
    Serial.println(state);
    return state;
}

/**
 * @brief (Model 2 controllers only) Reads the door sensor to determine the
 * door state for door 2.
 */
#ifdef CG_MODEL_2
String deviceStatus2() {
    String state = "AJAR";
    if (openSensor2.isOpen()) {
        state = "OPEN";
    }
    else if (closeSensor2.isOpen()) {
        state = "CLOSED";
    }

    Serial.print(F("INFO: Door status = "));
    Serial.println(state);
    return state;
}
#endif

/**
 * @brief Gets the current time as a string.
 * @return String The current date/time.
 */
String getCurrentTime() {
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    return String(asctime(timeinfo));
}

/**
 * @brief Synchronize the local system clock via NTP. Note: This does not take DST
 * into account. Currently, you will have to adjust the CLOCK_TIMEZONE define
 * manually to account for DST when needed.
 */
void onSyncClock() {
    configTime(TZ_America_New_York, "pool.ntp.org");

    Serial.print("INIT: Waiting for NTP time sync...");
    delay(500);
    while (!time(nullptr)) {
        ESPCrashMonitor.iAmAlive();
        Serial.print(F("."));
        delay(500);
    }
    
    Serial.println(F(" DONE"));
    Serial.print(F("INFO: Current time: "));
    Serial.println(getCurrentTime());
}

/**
 * @brief Enables MQTT message processing.
 */
void onEnableMqtt() {
    Serial.println(F("INIT: Disabling MQTT message processing deferral flag."));
    ignoreMqtt = false;
}

/**
 * @brief Publishes the system state to the MQTT status topic and
 * blinks the WiFi status LED.
 */
void publishSystemState() {
    if (mqttClient.connected()) {
        wifiLED.on();

        DynamicJsonDocument doc(200);
        doc["client_id"] = config.hostname;
        #ifdef CG_MODEL_2
            doc["door1State"] = deviceStatus();
            doc["door2State"] = deviceStatus2();
        #else
            doc["doorState"] = deviceStatus();
        #endif
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["systemState"] = (uint8_t)sysState;

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(config.mqttTopicStatus.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        wifiLED.off();
    }
}

/**
 * @brief Publishes a discovery packet to the configured discovery topic if
 * currently connected to WiFi.
 */
void publishDiscoveryPacket() {
    if (mqttClient.connected()) {
        wifiLED.on();

        DynamicJsonDocument doc(250);
        doc["name"] = config.hostname;
        doc["class"] = DEVICE_CLASS;
        doc["statusTopic"] = config.mqttTopicStatus;
        doc["controlTopic"] = config.mqttTopicControl;

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing discovery packet: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(config.mqttTopicStatus.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        wifiLED.off();
    }
}

/**
 * @brief Resume normal operation. This will resume any suspended tasks.
 */
void resumeNormal() {
    Serial.println(F("INFO: Resuming normal operation..."));
    taskMan.enableAll();
    wifiLED.off();
    sysState = SystemState::NORMAL;
    publishSystemState();
}

/**
 * @brief Prints network information details to the serial console.
 */
void printNetworkInfo() {
    Serial.print(F("INFO: Local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("INFO: Gateway: "));
    Serial.println(WiFi.gatewayIP());
    Serial.print(F("INFO: Subnet mask: "));
    Serial.println(WiFi.subnetMask());
    Serial.print(F("INFO: DNS server: "));
    Serial.println(WiFi.dnsIP());
    Serial.print(F("INFO: MAC address: "));
    Serial.println(WiFi.macAddress());
    WiFi.printDiag(Serial);
}

/**
 * @brief Scan for available networks and dump each discovered network to the
 * console.
 */
void getAvailableNetworks() {
    ESPCrashMonitor.defer();
    Serial.println(F("INFO: Scanning WiFi networks..."));
    int numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        Serial.print(F("ID: "));
        Serial.print(i);
        Serial.print(F("\tNetwork name: "));
        Serial.print(WiFi.SSID(i));
        Serial.print(F("\tSignal strength:"));
        Serial.println(WiFi.RSSI(i));
    }
    Serial.println(F("----------------------------------"));
}

/**
 * @brief Reboots the MCU after a 1 second delay.
 */
void reboot() {
    Serial.println(F("INFO: Rebooting... "));
    Serial.flush();
    delay(1000);
    ResetManager.softReset();
}

/**
 * @brief Stores the in-memory configuration to a JSON file stored in SPIFFS.
 * If the file does not yet exist, it will be created (see CONFIG_FILE_PATH).
 * Errors will be reported to the serial console if the filesystem is not
 * mounted or if the file could not be opened for writing.
 */
void saveConfiguration() {
    Serial.print(F("INFO: Saving configuration to: "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.println(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    StaticJsonDocument<350> doc;
    doc["hostname"] = config.hostname;
    doc["useDHCP"] = config.useDhcp;
    doc["ip"] = config.ip.toString();
    doc["gateway"] = config.gw.toString();
    doc["subnetMask"] = config.sm.toString();
    doc["dnsServer"] = config.dns.toString();
    doc["wifiSSID"] = config.ssid;
    doc["wifiPassword"] = config.password;
    doc["mqttBroker"] = config.mqttBroker;
    doc["mqttPort"] = config.mqttPort;
    doc["mqttControlTopic"] = config.mqttTopicControl;
    doc["mqttStatusTopic"] = config.mqttTopicStatus;
    doc["mqttDiscoveryTopic"] = config.mqttTopicDiscovery;
    doc["mqttUsername"] = config.mqttUsername;
    doc["mqttPassword"] = config.mqttPassword;
    #ifdef ENABLE_OTA
        doc["otaPort"] = config.otaPort;
        doc["otaPassword"] = config.otaPassword;
    #endif

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "w");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to open config file for writing."));
        doc.clear();
        return;
    }

    serializeJsonPretty(doc, configFile);
    doc.clear();
    configFile.flush();
    configFile.close();
    Serial.println(F("DONE"));
}

/**
 * @brief Helper method for printing a warning message during config load.
 * @param message The message to print.
 */
void printWarningAndContinue(const __FlashStringHelper *message) {
    Serial.println();
    Serial.println(message);
    Serial.print(F("INFO: Continuing... "));
}

/**
 * @brief Sets the running config to default configuration values.
 */
void setConfigurationDefaults() {
    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = defHostname;
    config.ip = defaultIp;
    config.mqttBroker = MQTT_BROKER;
    config.mqttPassword = "";
    config.mqttPort = MQTT_PORT;
    config.mqttTopicControl = MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = MQTT_TOPIC_STATUS;
    config.mqttTopicDiscovery = MQTT_TOPIC_DISCOVERY;
    config.mqttUsername = "";
    config.password = DEFAULT_PASSWORD;
    config.sm = defaultSm;
    config.ssid = DEFAULT_SSID;
    config.useDhcp = false;
    config.clockTimezone = CLOCK_TIMEZONE;
    config.dns = defaultDns;
    config.gw = defaultGw;

    #ifdef ENABLE_OTA
        config.otaPassword = OTA_PASSWORD;
        config.otaPort = OTA_HOST_PORT;
    #endif
}

/**
 * @brief Loads the configuration from CONFIG_FILE_PATH into memory and uses that as
 * the running configuration. Will report errors to the serial console and
 * revert to the default configuration under the following conditions:
 * 1) The filesystem is not mounted.
 * 2) The config file does not exist in SPIFFS. In this case a new file
 * will be created and populated with the default configuration.
 * 3) The config file exists, but could not be opened for reading.
 * 4) The config file is too big ( > 1MB).
 * 5) The config file could not be deserialized to a JSON structure.
 */
void loadConfiguration() {
    memset(&config, 0, sizeof(config));

    Serial.print(F("INFO: Loading config file: "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.print(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    if (!SPIFFS.exists(CONFIG_FILE_PATH)) {
        Serial.println(F("FAIL"));
        Serial.println(F("WARN: Config file does not exist. Creating with default config..."));
        saveConfiguration();
        return;
    }

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "r");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to open config file. Using default config."));
        return;
    }

    size_t size = configFile.size();
    uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;
    if (size > freeMem) {
        Serial.println(F("FAIL"));
        Serial.print(F("ERROR: Not enough free memory to load document. Size = "));
        Serial.print(size);
        Serial.print(F(", Free = "));
        Serial.println(freeMem);
        configFile.close();
        return;
    }

    DynamicJsonDocument doc(freeMem);
    DeserializationError error = deserializeJson(doc, configFile);
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to parse config file to JSON. Using default config."));
        configFile.close();
        return;
    }

    doc.shrinkToFit();
    configFile.close();

    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = doc.containsKey("hostname") ? doc["hostname"].as<String>() : defHostname;
    config.useDhcp = doc.containsKey("useDhcp") ? doc["useDhcp"].as<bool>() : false;
    if (doc.containsKey("ip")) {
        if (!config.ip.fromString(doc["ip"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.ip = defaultIp;
    }
    
    if (doc.containsKey("gateway")) {
        if (!config.gw.fromString(doc["gateway"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
        }
    }
    else {
        config.gw = defaultGw;
    }
    
    if (doc.containsKey("subnetmask")) {
        if (!config.sm.fromString(doc["subnetMask"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid subnet mask in configuration. Falling back to default."));
        }
    }
    else {
        config.sm = defaultSm;
    }

    if (doc.containsKey("dns")) {
        if (!config.dns.fromString(doc["dnsServer"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid DSN server in configuration. Falling back to default."));
        }
    }
    else {
        config.dns = defaultDns;
    }
    
    config.ssid = doc.containsKey("wifiSSID") ? doc["wifiSSID"].as<String>() : DEFAULT_SSID;
    config.password = doc.containsKey("wifiPassword") ? doc["wifiPassword"].as<String>() : DEFAULT_PASSWORD;
    config.mqttBroker = doc.containsKey("mqttBroker") ? doc["mqttBroker"].as<String>() : MQTT_BROKER;
    config.mqttPort = doc.containsKey("mqttPort") ? doc["mqttPort"].as<int>() : MQTT_PORT;
    config.mqttTopicControl = doc.containsKey("mqttControlTopic") ? doc["mqttControlTopic"].as<String>() : MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = doc.containsKey("mqttStatusTopic") ? doc["mqttStatusTopic"].as<String>() : MQTT_TOPIC_STATUS;
    config.mqttTopicDiscovery = doc.containsKey("mqttDiscoveryTopic") ? doc["mqttDiscoveryTopic"].as<String>() : MQTT_TOPIC_DISCOVERY;
    config.mqttUsername = doc.containsKey("mqttUsername") ? doc["mqttUsername"].as<String>() : "";
    config.mqttPassword = doc.containsKey("mqttPassword") ? doc["mqttPassword"].as<String>() : "";
    #ifdef ENABLE_OTA
        config.otaPort = doc.containsKey("otaPort") ? doc["otaPort"].as<uint16_t>() : MQTT_PORT;
        config.otaPassword = doc.containsKey("otaPassword") ? doc["otaPassword"].as<String>() : OTA_PASSWORD;
    #endif

    doc.clear();
    Serial.println(F("DONE"));
}

/**
 * @brief Confirms with the user that they wish to do a factory restore. If so, then
 * clears the current configuration file in SPIFFS, then reboots. Upon reboot,
 * a new config file will be generated with default values.
 */
void doFactoryRestore() {
    Serial.println();
    Serial.println(F("Are you sure you wish to restore to factory default? (Y/n)?"));
    Console.waitForUserInput();

    String str = Console.getInputString();
    if (str == "Y" || str == "y") {
        Serial.print(F("INFO: Clearing current config... "));
        if (filesystemMounted) {
            if (SPIFFS.remove(CONFIG_FILE_PATH)) {
                Serial.println(F("DONE"));
                Serial.print(F("INFO: Removed file: "));
                Serial.println(CONFIG_FILE_PATH);

                Serial.print(F("INFO: Rebooting in "));
                for (uint8_t i = 5; i >= 1; i--) {
                    Serial.print(i);
                    Serial.print(F(" "));
                    delay(1000);
                }

                reboot();
            }
            else {
                Serial.println(F("FAIL"));
                Serial.println(F("ERROR: Failed to delete configuration file."));
            }
        }
        else {
            Serial.println(F("FAIL"));
            Serial.println(F("ERROR: Filesystem not mounted."));
        }
    }

    Serial.println();
}

/**
 * @brief Attempts to re-connect to the MQTT broker if then connection is
 * currently broken.
 * @return true if a connection to the MQTT broker is either already
 * established, or was successfully re-established; Otherwise, false.
 * If the connection is re-established, then will also re-subscribe to
 * the status channel.
 */
bool reconnectMqttClient() {
    if (!mqttClient.connected()) {
        Serial.print(F("INFO: Attempting to establish MQTT connection to "));
        Serial.print(config.mqttBroker);
        Serial.print(F(" on port: "));
        Serial.print(config.mqttPort);
        Serial.println(F("..."));
        
        bool didConnect = false;
        if (config.mqttUsername.length() > 0 && config.mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(config.hostname.c_str(), config.mqttUsername.c_str(), config.mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(config.hostname.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to topic: "));
            Serial.println(config.mqttTopicControl);
            mqttClient.subscribe(config.mqttTopicControl.c_str());

            Serial.print(F("INFO: Publishing to topic: "));
            Serial.println(config.mqttTopicStatus);

            Serial.print(F("INFO: Discovery topic: "));
            Serial.println(config.mqttTopicDiscovery);
        }
        else {
            String failReason = TelemetryHelper::getMqttStateDesc(mqttClient.state());
            Serial.print(F("ERROR: Failed to connect to MQTT broker: "));
            Serial.println(failReason);
            return false;
        }
    }

    return true;
}

/**
 * @brief Callback method for checking the MQTT connection state and
 * reconnecting if necessary. If the connection is broken, and
 * reconnection fails, another attempt will be made after CHECK_MQTT_INTERVAL.
 */
void onCheckMqtt() {
    Serial.println(F("INFO: Checking MQTT connection status..."));
    if (reconnectMqttClient()) {
        Serial.println(F("INFO: Successfully reconnected to MQTT broker."));
        publishSystemState();
        publishDiscoveryPacket();
    }
    else {
        Serial.println(F("ERROR: MQTT connection lost and reconnect failed."));
        Serial.print(F("INFO: Retrying connection in "));
        Serial.print(CHECK_MQTT_INTERVAL % 1000);
        Serial.println(F(" seconds."));
    }
}

/**
 * @brief Activates the door control relay, sends response to HTTP client, and
 * blinks WiFi LED to indicate activity.
 */
void activateDoor() {
    Serial.print(F("INFO: Activating door "));
    #ifdef CG_MODEL_2
        Serial.print(F(" 1 "));
    #endif
    Serial.print(F("@ "));
    Serial.println(getCurrentTime());
    wifiLED.on();
    garageDoorRelay.close();
    publishSystemState();
    wifiLED.off();
}

#ifdef CG_MODEL_2
void activateDoor2() {
    Serial.print(F("INFO: Activating door 2 @ "));
    Serial.println(getCurrentTime());
    wifiLED.on();
    garageDoorRelay2.close();
    publishSystemState();
    wifiLED.off();
}
#endif

/**
 * @brief Processes control requests. Executes the specified command
 * if valid and intended for this client.
 * @param id The client ID the command is intended for.
 * @param cmd The command to execute.
 */
void handleControlRequest(String id, ControlCommand cmd) {
    id.toUpperCase();
    if (!id.equals(config.hostname)) {
        Serial.println(F("INFO: Control message not intended for this host. Ignoring..."));
        return;
    }

    // When system is the "disabled" state, the only command it will accept
    // is "enable". All other commands are ignored.
    if (sysState == SystemState::DISABLED &&
        cmd != ControlCommand::ENABLE) {
        // THOU SHALT NOT PASS!!! 
        // We can't process this command because we are disabled.
        Serial.print(F("WARN: Ingoring command "));
        Serial.print((uint8_t)cmd);
        Serial.print(F(" because the system is currently disabled."));
        return;
    }

    switch (cmd) {
        case ControlCommand::ACTIVATE:
            activateDoor();
            break;
        #ifdef CG_MODEL_2
        case ControlCommand::ACTIVATE_2:
            activateDoor2();
            break;
        #endif
        case ControlCommand::DISABLE:
            Serial.println(F("WARN: Disabling system."));
            sysState = SystemState::DISABLED;
            break;
        case ControlCommand::ENABLE:
            Serial.println(F("INFO: Enabling system."));
            sysState = SystemState::NORMAL;
            break;
        case ControlCommand::REBOOT:
            reboot();
            break;
        case ControlCommand::REQUEST_STATUS:
            break;
        default:
            Serial.print(F("WARN: Unknown command: "));
            Serial.println((uint8_t)cmd);
            break;
    }

    publishSystemState();
}

/**
 * @brief Callback method for handling incoming MQTT messages on the subscribed
 * channel(s). This will decode the message and process incoming commands.
 * Commands are only executed if the command was intended for this device.
 * The system makes this determination by comaparing the ID in the JSON
 * payload with the systems currenty device ID (host name).
 * @param topic The topic the message came in on.
 * @param payload The message payload. The payload is assumed to be in
 * JSON format and will attempt to parse the message into a JSON object.
 * @param length The payload (message) length.
 */
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print(F("INFO: [MQTT] ["));
    Serial.print(getCurrentTime());
    Serial.print(F("] Message arrived: ["));
    Serial.print(topic);
    Serial.print(F("] "));

    // It's a lot easier to deal with if we just convert the payload
    // to a string first.
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.println(msg);
    if (sysState == SystemState::BOOTING || ignoreMqtt) {
        Serial.println(F("INIT: Boot sequence incomplete or message processing temporarily disabled."));
        return;
    }

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.print(F("ERROR: Failed to parse MQTT message to JSON: "));
        Serial.println(error.c_str());
        doc.clear();
        return;
    }

    String id = doc["client_id"].as<String>();
    ControlCommand cmd = (ControlCommand)doc["command"].as<uint8_t>();
    doc.clear();
    handleControlRequest(id, cmd);
}

/**
 * @brief Initializes the MDNS responder (if enabled).
 */
void initMDNS() {
    #ifdef ENABLE_MDNS
        Serial.print(F("INIT: Starting MDNS responder... "));
        if (WiFi.status() == WL_CONNECTED) {
            ESPCrashMonitor.defer();
            delay(500);
            if (!mdns.begin(config.hostname)) {
                Serial.println(F(" FAILED"));
                return;
            }

            #ifdef ENABLE_OTA
                mdns.addService(config.hostname, "ota", config.otaPort);
            #endif
            Serial.println(F(" DONE"));
        }
        else {
            Serial.println(F(" FAILED"));
        }
    #endif
}

/**
 * @brief Initialize the SPIFFS filesystem.
 */
void initFilesystem() {
    Serial.print(F("INIT: Initializing SPIFFS and mounting filesystem... "));
    if (!SPIFFS.begin()) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to mount filesystem."));
        return;
    }

    filesystemMounted = true;
    Serial.println(F("DONE"));
    setConfigurationDefaults();
    loadConfiguration();
}

/**
 * @brief Initializes the MQTT client.
 */
void initMQTT() {
    Serial.print(F("INIT: Initializing MQTT client... "));
    mqttClient.setServer(config.mqttBroker.c_str(), config.mqttPort);
    mqttClient.setCallback(onMqttMessage);
    mqttClient.setBufferSize(500);
    Serial.println(F("DONE"));
    if (reconnectMqttClient()) {
        delay(500);
        publishSystemState();
    }
}

/**
 * @brief Attempt to connect to the configured WiFi network. This will break any
 * existing connection first.
 */
void connectWifi() {
    Serial.println(F("DEBUG: Setting mode..."));
    WiFi.mode(WIFI_STA);
    Serial.println(F("DEBUG: Disconnect and clear to prevent auto connect..."));
    WiFi.persistent(false);
    WiFi.disconnect(true);
    ESPCrashMonitor.defer();

    delay(1000);
    if (config.useDhcp) {
        WiFi.config(0U, 0U, 0U, 0U);
    }
    else {
        WiFi.config(config.ip, config.gw, config.sm, config.gw);
    }

    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(config.hostname, config.password);
    Serial.println(F("DEBUG: Waiting for connection..."));
    
    const int maxTries = 20;
    int currentTry = 0;
    while ((WiFi.status() != WL_CONNECTED) && (currentTry < maxTries)) {
        ESPCrashMonitor.iAmAlive();
        currentTry++;
        wifiLED.blink(500);
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        // Connection failed. Maybe the AP went down? Let's try again later.
        Serial.println(F("ERROR: Failed to connect to WiFi!"));
        Serial.println(F("WARN: Will attempt to reconnect at scheduled interval."));
    }
    else {
        printNetworkInfo();
    }
}

/**
 * @brief Enter fail-safe mode. This will suspend all tasks, disable relay activation,
 * and propmpt the user for configuration.
 */
void failSafe() {
    sysState = SystemState::DISABLED;
    publishSystemState();
    ESPCrashMonitor.defer();
    Serial.println();
    Serial.println(F("ERROR: Entering failsafe (config) mode..."));
    taskMan.disableAll();
    sensorLED.on();
    wifiLED.on();
    garageDoorRelay.open();
    #ifdef CG_MODEL_2
        garageDoorRelay2.open();
    #endif
    Console.enterCommandInterpreter();
}

/**
 * @brief Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to SSID: "));
    Serial.print(config.ssid);
    Serial.println(F("..."));
    
    connectWifi();
}

/**
 * @brief Initializes the OTA update listener if enabled.
 */
void initOTA() {
    #ifdef ENABLE_OTA
        Serial.print(F("INIT: Starting OTA updater... "));
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.setPort(config.otaPort);
            ArduinoOTA.setHostname(config.hostname.c_str());
            ArduinoOTA.setPassword(config.otaPassword.c_str());
            ArduinoOTA.onStart([]() {
                // Handles start of OTA update. Determines update type.
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                }
                else {
                    type = "filesystem";
                }

                sysState = SystemState::UPDATING;
                publishSystemState();
                Serial.println("INFO: Starting OTA update (type: " + type + ") ...");
            });
            ArduinoOTA.onEnd([]() {
                // Handles update completion.
                Serial.println(F("INFO: OTA updater stopped."));
            });
            ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
                // Reports update progress.
                wifiLED.blink(100);
                ESPCrashMonitor.iAmAlive();
                Serial.printf("INFO: OTA Update Progress: %u%%\r", (progress / (total / 100)));
            });
            ArduinoOTA.onError([](ota_error_t error) {
                // Handles OTA update errors.
                Serial.printf("ERROR: OTA update error [%u]: ", error);
                switch(error) {
                    case OTA_AUTH_ERROR:
                        Serial.println(F("Auth failed."));
                        break;
                    case OTA_BEGIN_ERROR:
                        Serial.println(F("Begin failed."));
                        break;
                    case OTA_CONNECT_ERROR:
                        Serial.println(F("Connect failed."));
                        break;
                    case OTA_RECEIVE_ERROR:
                        Serial.println(F("Receive failed."));
                        break;
                    case OTA_END_ERROR:
                        Serial.println(F("End failed."));
                        break;
                }
            });
            ArduinoOTA.begin();
            Serial.println(F("DONE"));
        }
        else {
            Serial.println(F("FAIL"));
        }
    #endif
}

/**
 * @brief Initializes the RS232 serial interface.
 */
void initSerial() {
    Serial.setDebugOutput(true);
    Serial.begin(SERIAL_BAUD, SERIAL_8N1);
    Serial.println();
    Serial.println();
    Serial.print(F("INIT: CyGarage v"));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F(" booting ..."));
    Serial.println();
}

/**
 * @brief Initializes the required input pins.
 */
void initInputs() {
    Serial.print(F("INIT: Initializing sensors... "));
    openSensor.init();
    closeSensor.init();
    #ifdef CG_MODEL_2
        openSensor2.init();
        closeSensor2.init();
    #endif
    Serial.println(F("DONE"));
}

/**
 * @brief Initializes output components.
 */
void initOutputs() {
    Serial.print(F("INIT: Initializing components... "));
    sensorLED.init();
    sensorLED.on();

    wifiLED.init();
    wifiLED.on();

    garageDoorRelay.init();
    garageDoorRelay.open();

    #ifdef CG_MODEL_2
        garageDoorRelay2.init();
        garageDoorRelay2.open();
    #endif

    delay(1000);
    Serial.println(F("DONE"));
}

/**
 * @brief Initializes the task manager and all recurring tasks.
 */
void initTaskManager() {
    Serial.print(F("INIT: Initializing task scheduler... "));

    taskMan.init();
    taskMan.addTask(tCheckActivation);
    taskMan.addTask(tCheckWifi);
    taskMan.addTask(tCheckMqtt);
    taskMan.addTask(tSyncClock);
    taskMan.addTask(tEnableMqtt);
    
    tCheckWifi.enableDelayed(30000);
    tCheckActivation.enable();
    tCheckMqtt.enableDelayed(1000);
    tSyncClock.enable();
    tEnableMqtt.enableDelayed(1000);
    Serial.println(F("DONE"));
}

/**
 * @brief Callback routine for checking WiFi connectivity. If not connected,
 * this will attempt a reconnect and if successful, will re-init MDNS and OTA.
 */
void onCheckWiFi() {
    Serial.println(F("INFO: Checking WiFi connectivity..."));
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WARN: Lost connection. Attempting reconnect..."));
        connectWifi();
        if (WiFi.status() == WL_CONNECTED) {
            initMDNS();
            initOTA();
        }
    }
}

/**
 * @brief Callback routine for checking if the door has been activated, and
 * deactivates it if it has.
 */
void onCheckActivation() {
    if (garageDoorRelay.isClosed()) {
        Serial.print(F("INFO: Deactivating door "));
        #ifdef CG_MODEL_2
            Serial.print(F("1 "));
        #endif
        Serial.print(F("@ "));
        Serial.println(millis());
        garageDoorRelay.open();
    }

    #ifdef CG_MODEL_2
        if (garageDoorRelay2.isClosed()) {
            Serial.print(F("INFO: Deactivating door 2 @ "));
            Serial.println(millis());
            garageDoorRelay2.open();
        }
    #endif
}

/**
 * @brief Callback routine that checks the sensor inputs to determine door state.
 */
void onCheckSensors() {
    Serial.println(F("INFO: Checking sensor..."));
    String state = deviceStatus();
    if (state == "AJAR" || state == "OPEN") {
        if (sensorLED.isOff()) {
            sensorLED.on();
        }
    }
    else {
        sensorLED.off();
    }

    if (!lastState.equals(state)) {
        lastState = state;
        publishSystemState();
    }

    #ifdef CG_MODEL_2
        String state2 = deviceStatus2();
        if (state2 == "AJAR" || state == "OPEN") {
            if (sensorLED.isOff()) {
                sensorLED.on();
            }
        }
        else {
            sensorLED.off();
        }

        if (!lastState2.equals(state2)) {
            lastState2 = state2;
            publishSystemState();
        }
    #endif
}

/**
 * @brief Callback handler for when a door contact changes state.
 * @param sender The door that changed state.
 */
void onDoorContactStateChange(DoorInfo* sender) {
    onCheckSensors();
}

/**
 * @brief Initializes the crash monitor and dump any previous crash data to the
 * serial console.
 */
void initCrashMonitor() {
    Serial.print(F("INIT: Initializing crash monitor... "));
    ESPCrashMonitor.disableWatchdog();
    Serial.println(F("DONE"));
    ESPCrashMonitor.dump(Serial);
    delay(100);
}

/**
 * @brief Callback handler for activating door 1 from the CLI.
 */
void onActivateDoor1() {
    activateDoor();
    delay(1000);
    garageDoorRelay.open();
    Console.enterCommandInterpreter();
}

/**
 * @brief (Only for Model 2 controllers) Callback handler for activating
 * door 2 from the CLI.
 */
#ifdef CG_MODEL_2
void onActivateDoor2() {
    activateDoor2();
    delay(1000);
    garageDoorRelay2.open();
    Console.enterCommandInterpreter();
}
#endif

/**
 * @brief Callback handler for performing a "factory restore" of the config
 * from the CLI. This executes the "factory restore" procedure.
 */
void handleFactoryRestore() {
    doFactoryRestore();
}

/**
 * @brief Callback handler for changing the localhost hostname from the CLI.
 * This will apply the new hostname to the running config and then re-init
 * the MDNS resolver.
 * @param newHostName The new hostname. 
 */
void onNewHostName(const char* newHostName) {
    if (config.hostname != newHostName) {
        config.hostname = newHostName;
        WiFi.setHostname(config.hostname.c_str());
        initMDNS();
    }
}

/**
 * @brief Callback handler for switching to DHCP mode from the CLI. This
 * will apply the DHCP flag to the running config and then force the WiFi
 * interface to request configuration from DHCP.
 */
void onSwitchToDhcp() {
    if (config.useDhcp) {
        Serial.println(F("INFO: DHCP mode already set. Skipping..."));
        Serial.println();
    }
    else {
        config.useDhcp = true;
        Serial.println(F("INFO: Set DHCP mode."));
        WiFi.config(0U, 0U, 0U, 0U);
    }
}

/**
 * @brief Callback handler for switching to or updating the static IP settings.
 * This will apply the new settings to the running config and then force the
 * WiFi interface into static IP mode using those settings.
 * @param newIp The new local IP address.
 * @param newSm The new subnet mask.
 * @param newGw The new gateway IP address.
 * @param newDns The new DNS IP address.
 */
void onSwitchToStaticConfig(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns) {
    config.ip = newIp;
    config.sm = newSm;
    config.gw = newGw;
    config.dns = newDns;

    // If actual IP set, then disables DHCP and assumes static.
    WiFi.config(config.ip, config.gw, config.sm, config.dns);
}

/**
 * @brief Callback handler for reconnecting to WiFi from the CLI. This will
 * execute a status check. If not connected, this will then attempt to
 * reconnect. If successful, this will automatically resume normal operation
 * and exit the CLI. If not successful, will report the error and remain in
 * the CLI.
 */
void onReconnectFromConsole() {
    // Attempt to reconnect to WiFi.
    onCheckWiFi();
    if (WiFi.status() == WL_CONNECTED) {
        printNetworkInfo();
        resumeNormal();
    }
    else {
        Serial.println(F("ERROR: Still no network connection."));
        Console.enterCommandInterpreter();
    }
}

/**
 * @brief Callback handler for changing the WiFi settings from the CLI. This
 * applies the new settings to the running config and then reconnects to WiFi.
 * @param newSsid The new SSID.
 * @param newPassword The new password.
 */
void onWifiConfig(String newSsid, String newPassword) {
    if (config.ssid != newSsid || config.password != newPassword) {
        config.ssid = newSsid;
        config.password = newPassword;
        connectWifi();
    }
}

/**
 * @brief Callback handler for persisting the running config to flash.
 */
void onSaveConfig() {
    saveConfiguration();
    WiFi.disconnect(true);
    onCheckWiFi();
}

/**
 * @brief Callback handler for when the user changes the MQTT settings from
 * the CLI. This unsubscribes from any subscribed topics, disconnects from
 * the current broker, then re-inits the MQTT client.
 * @param newBroker The new MQTT broker.
 * @param newPort The new MQTT port.
 * @param newUsername The new username.
 * @param newPass The new password.
 * @param newConChan The new control topic.
 * @param newStatChan The new status topic.
 */
void onMqttConfigCommand(String newBroker, int newPort, String newUsername, String newPass, String newConChan, String newStatChan) {
    mqttClient.unsubscribe(config.mqttTopicControl.c_str());
    mqttClient.disconnect();

    config.mqttBroker = newBroker;
    config.mqttPort = newPort;
    config.mqttUsername = newUsername;
    config.mqttPassword = newPass;
    config.mqttTopicControl = newConChan;
    config.mqttTopicStatus = newStatChan;

    initMQTT();
    Serial.println();
}

/**
 * @brief Initialize the CLI.
 */
void initConsole() {
    Serial.print(F("INIT: Initializing console... "));
    Console.setHostName(config.hostname);
    Console.setMqttConfig(config.mqttBroker,
        config.mqttPort,
        config.mqttUsername,
        config.mqttPassword,
        config.mqttTopicControl,
        config.mqttTopicStatus);

    Console.onActivateDoor1(onActivateDoor1);
    #ifdef CG_MODEL_2
    Console.onActivateDoor2(onActivateDoor2);
    #endif
    Console.onRebootCommand(reboot);
    Console.onScanNetworks(getAvailableNetworks);
    Console.onHostNameChange(onNewHostName);
    Console.onDhcpConfig(onSwitchToDhcp);
    Console.onStaticConfig(onSwitchToStaticConfig);
    Console.onReconnectCommand(onReconnectFromConsole);
    Console.onWiFiConfigCommand(onWifiConfig);
    Console.onResumeCommand(resumeNormal);
    Console.onGetNetInfoCommand(printNetworkInfo);
    Console.onSaveConfigCommand(onSaveConfig);
    Console.onMqttConfigCommand(onMqttConfigCommand);
    Console.onConsoleInterrupt(failSafe);
    Console.onFactoryRestore(handleFactoryRestore);

    Serial.println(F("DONE"));
}

/**
 * @brief Bootstrap routine. Executes once at boot and initializes all
 * subsystems in sequence.
 */
void setup() {
    // This boot sequence is important. DO NOT ALTER.
    initSerial();
    initCrashMonitor();
    initOutputs();
    initInputs();
    initFilesystem();
    initWiFi();
    initMDNS();
    initOTA();
    initMQTT();
    initTaskManager();
    initConsole();
    Serial.println(F("INIT: Boot sequence complete."));
    sysState = SystemState::NORMAL;
    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
}

/**
 * @brief Main loop. Executes all tasks that need to be, handles incoming web server requests,
 * and OTA update requests.
 */
void loop() {
    ESPCrashMonitor.iAmAlive();
    Console.checkInterrupt();
    taskMan.execute();
    #ifdef ENABLE_MDNS
        mdns.update();
    #endif
    #ifdef ENABLE_OTA
        ArduinoOTA.handle();
    #endif
    mqttClient.loop();
}