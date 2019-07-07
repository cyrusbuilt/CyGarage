/**
 * main.cpp
 * CyGarage v1.0
 * 
 * (c) 2019, Cyrus Brunner
 * 
 * This is the firmware for CyGarage, an IoT garage door opener add-on. The devie is intended
 * to integrate with an existing garage door opener and connect via WiFi, providing the ability
 * to sensor when the door is open/closed/ajar and provide a means of opening/closing the door.
 * This firmware also allows for OTA firmware updates, and additionally can be integrated with
 * a home automation server such as OpenHAB.
 */

#ifndef ESP8266
    #error This firmware is only compatible with ESP8266 controllers.
#endif
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include "DoorContact.h"
#include "LED.h"
#include "Relay.h"
#include "TaskScheduler.h"
#include "ResetManager.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
extern "C" {
  #include "user_interface.h"
}

#define FIRMWARE_VERSION "1.0"

// Configuration
#define ENABLE_OTA                              // Comment this line to disable OTA updates.
#define ENABLE_MDNS                             // Comment this line to disable the MDNS.
#define DEFAULT_SSID "your_ssid_here"           // Put the SSID of your WiFi here.
#define DEFAULT_PASSWORD "your_password_here"   // Put your WiFi password here.
#define WEBSERVER_PORT 80                       // The built-in webserver port.
#define SERIAL_BAUD 115200                      // The BAUD rate (speed) of the serial port (console).
#define CHECK_WIFI_INTERVAL 30000               // How often to check WiFi status (milliseconds).
#define CHECK_SENSORS_INTERVAL 3000             // How often to check sensors (milliseconds).
#define ACTIVATION_DURATION 1000                // How long the activation relay should be on.
#define DEVICE_NAME "CyGarage"                  // The device name.
#ifdef ENABLE_OTA
    #define OTA_HOST_PORT 8266                     // The OTA updater port.
    #define OTA_HOSTNAME DEVICE_NAME               // The OTA updater host name. Should match device name.
    #define OTA_PASSWORD "your_ota_password_here"  // The OTA updater password.
#endif
IPAddress ip(192, 168, 0, 141);                 // The default static host IP.
IPAddress gw(192, 168, 0, 1);                   // The default static gateway IP.
IPAddress sm(255, 255, 255, 0);                 // The default static subnet mask.

// Pin definitions
#define PIN_SENSOR_LED 5
#define PIN_WIFI_LED 2
#define PIN_ACTIVATE 4
#define PIN_OPEN_SENSOR 12
#define PIN_CLOSE_SENSOR 13

// Forward declarations
void onRelayStateChange(RelayInfo* sender);
void onCheckWiFi();
void onCheckActivation();
void onCheckSensors();
void failSafe();


// Global vars
#ifdef ENABLE_MDNS
MDNSResponder mdns;
#endif
ESP8266WebServer server(WEBSERVER_PORT);
// TODO we can probably refactor to use the new DoorContact's events and loop() method,
// instead of using a scheduled task to periodically poll. We can keep as-is for now
// though. Nothing 'wrong' with current implementation. Just might not be the best.
DoorContact openSensor(PIN_OPEN_SENSOR, NULL);
DoorContact closeSensor(PIN_CLOSE_SENSOR, NULL);
Relay garageDoorRelay(PIN_ACTIVATE, onRelayStateChange, "GARAGE_DOOR");
LED sensorLED(PIN_SENSOR_LED, NULL);
LED wifiLED(PIN_WIFI_LED, NULL);
Task tCheckWifi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWiFi);
Task tCheckActivation(ACTIVATION_DURATION, TASK_FOREVER, &onCheckActivation);
Task tCheckSensors(CHECK_SENSORS_INTERVAL, TASK_FOREVER, &onCheckSensors);
Scheduler taskMan;
String ssid = DEFAULT_SSID;
String password = DEFAULT_PASSWORD;
bool isDHCP = false;
bool wasDHCP = false;

/**
 * Waits for user input from the serial console.
 */
void waitForUserInput() {
    while (Serial.available() < 1) {
        ESPCrashMonitor.iAmAlive();
        delay(50);
    }
}

/**
 * Gets string input from the serial console.
 */
String getInputString() {
    char c;
    String result = "";
    bool gotEndMarker = false;
    while (!gotEndMarker) {
        ESPCrashMonitor.iAmAlive();
        if (Serial.available() > 0) {
            c = Serial.read();
            if (c == '\n') {
                gotEndMarker = true;
                break;
            }

            result += c;
        }
    }

    return result;
}

/**
 * Prints network information details to the serial console.
 */
void printNetworkInfo() {
    Serial.print(F("INFO: Local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("INFO: MAC address: "));
    Serial.println(WiFi.macAddress());
    WiFi.printDiag(Serial);
}

/**
 * Scan for available networks and dump each discovered network to the console.
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
 * Read the specified sensor pins and determine the door state. Prints
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

    Serial.print(F("Door status = "));
    Serial.println(state);
    return state;
}

/**
 * Gets the device status and sends it via HTTP to connected clients.
 * Blinks the WiFi LED to indicate activity.
 */
void getDeviceStatus() {
    wifiLED.on();
    server.send(200, "text/plain", deviceStatus());
    wifiLED.off();
}

/**
 * Gets the firmware version and sends it via HTTP to connected clients.
 * Blinks the WiFi LED to indicate activity.
 */
void getFirmwareVersion() {
    wifiLED.on();
    server.send(200, "text/plain", FIRMWARE_VERSION);
    wifiLED.off();
}

/**
 * Sends "Server OK" to connected clients over HTTP and blinks WiFi LED to
 * indicate activity.
 */
void serverOk() {
    wifiLED.on();
    server.send(200, "text/plain", "Server Ok...");
    wifiLED.off();
}

/**
 * Activates the door control relay, sends response to HTTP client, and
 * blinks WiFi LED to indicate activity.
 */
void activateDoor() {
    Serial.print(F("INFO: Activating door @ "));
    Serial.println(millis());
    garageDoorRelay.close();
    wifiLED.on();
    server.send(200, "text/plain", "OK");
    wifiLED.off();
}

/**
 * Initializes the MDNS responder (if enabled).
 */
void initMDNS() {
    #ifdef ENABLE_MDNS
        Serial.print(F("INIT: Starting MDNS responder... "));
        if (WiFi.status() == WL_CONNECTED) {
            ESPCrashMonitor.defer();
            delay(500);
            if (!mdns.begin(DEVICE_NAME)) {
                Serial.println(F(" FAILED"));
                return;
            }

            mdns.addService(DEVICE_NAME, "http", WEBSERVER_PORT);
            mdns.addService(OTA_HOSTNAME, "ota", OTA_HOST_PORT);
            Serial.println(F(" DONE"));
        }
        else {
            Serial.println(F(" FAILED"));
        }
    #endif
}

/**
 * Initializes the built-in web server.
 */
void initWebServer() {
    Serial.print(F("INIT: Initializing web server... "));
    if (WiFi.status() == WL_CONNECTED) {
        server.on("/", serverOk);
        server.on("/device/status", HTTP_GET, getDeviceStatus);
        server.on("/device/activate", HTTP_POST, activateDoor);
        server.on("/version", HTTP_GET, getFirmwareVersion);
        server.begin();
        Serial.println(F("DONE"));
    }
    else {
        Serial.println(F("FAIL"));
    }
}

/**
 * Prompts the user with a configuration screen and waits for
 * user input.
 */
void promptConfig() {
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F("= Command menu:              ="));
    Serial.println(F("=                            ="));
    Serial.println(F("= r: Reboot                  ="));
    Serial.println(F("= c: Configure network       ="));
    Serial.println(F("= s: Scan wireless networks  ="));
    Serial.println(F("= n: Connect to new network  ="));
    Serial.println(F("= w: Reconnect to WiFi       ="));
    Serial.println(F("= e: Resume normal operation ="));
    Serial.println(F("= g: Get network info        ="));
    Serial.println(F("= a: Activate door           ="));
    Serial.println(F("=                            ="));
    Serial.println(F("=============================="));
    Serial.println();
    Serial.println(F("Enter command choice (r/c/s/n/w/e/g): "));
    waitForUserInput();
}

/**
 * Gets an IPAddress value from the specified string.
 * @param value The string containing the IP.
 * @return The IP address.
 */
IPAddress getIPFromString(String value) {
    unsigned int ip[4];
    unsigned char buf[value.length()];
    value.getBytes(buf, value.length());
    const char* ipBuf = (const char*)buf;
    sscanf(ipBuf, "%u.%u.%u.%u", &ip[0], &ip[1], &ip[2], &ip[3]);
    return IPAddress(ip[0], ip[1], ip[2], ip[3]);
}

/**
 * Resume normal operation. This will resume any suspended tasks.
 */
void resumeNormal() {
    Serial.println(F("INFO: Resuming normal operation..."));
    taskMan.enableAll();
}

/**
 * Attempt to connect to the configured WiFi network. This will break any existing connection first.
 */
void connectWifi() {
    Serial.println(F("DEBUG: Setting mode..."));
    WiFi.mode(WIFI_STA);
    Serial.println(F("DEBUG: Disconnect and clear to prevent auto connect..."));
    WiFi.persistent(false);
    WiFi.disconnect(true);
    ESPCrashMonitor.defer();
    delay(1000);
    if (isDHCP) {
        WiFi.config(0U, 0U, 0U);
    }
    else {
        WiFi.config(ip, gw, sm);
    }
    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(ssid.c_str(), password.c_str());
    
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
        Serial.println(F("ERROR: Failed to connect to WiFi!"));
        failSafe();
    }
    else {
        printNetworkInfo();
    }
}

/**
 * Checks commands entered by the user via serial input and carries out
 * the specified action if valid.
 */
void checkCommand() {
    IPAddress addr(0, 0, 0, 0);
    String str = "";
    char incomingByte = Serial.read();
    switch (incomingByte) {
        case 'r':
            // Reset the controller.
            ResetManager.softReset();
            break;
        case 's':
            // Scan for available networks.
            getAvailableNetworks();
            promptConfig();
            checkCommand();
            break;
        case 'c':
            // Change network mode.
            Serial.println(F("Choose network mode (d = DHCP, t = Static):"));
            waitForUserInput();
            checkCommand();
            break;
        case 'd':
            // Switch to DHCP mode.
            WiFi.config(0U, 0U, 0U);
            isDHCP = true;
            Serial.println(F("INFO: Set DHCP mode."));
            promptConfig();
            checkCommand();
            break;
        case 't':
            // Switch to static IP mode. Request IP settings.
            isDHCP = false;
            Serial.println(F("Enter IP address: "));
            waitForUserInput();
            str = Serial.readStringUntil('\n');
            ip = getIPFromString(str);
            Serial.print(F("New IP: "));
            Serial.println(ip);
            Serial.println(F("Enter gateway: "));
            waitForUserInput();
            str = getInputString();
            gw = getIPFromString(str);
            Serial.print(F("New gateway: "));
            Serial.println(gw);
            Serial.println(F("Enter subnet mask: "));
            waitForUserInput();
            str = getInputString();
            sm = getIPFromString(str);
            Serial.print(F("New subnet mask: "));
            Serial.println(sm);
            WiFi.config(ip, gw, sm);
            promptConfig();
            checkCommand();
            break;
        case 'w':
            // Attempt to reconnect to WiFi.
            onCheckWiFi();
            if (WiFi.status() == WL_CONNECTED) {
                printNetworkInfo();
                resumeNormal();
            }
            else {
                Serial.println(F("ERROR: Still no network connection."));
                promptConfig();
                checkCommand();
            }
            break;
        case 'n':
            // Connect to a new wifi network.
            Serial.println(F("Enter new SSID: "));
            waitForUserInput();
            ssid = getInputString();
            Serial.print(F("SSID = "));
            Serial.println(ssid);
            Serial.println(F("Enter new password: "));
            waitForUserInput();
            password = getInputString();
            Serial.print(F("Password = "));
            Serial.println(password);
            connectWifi();
            break;
        case 'e':
            // Resume normal operation.
            resumeNormal();
            break;
        case 'g':
            // Get network info.
            printNetworkInfo();
            promptConfig();
            checkCommand();
            break;
        case 'a':
            activateDoor();
            delay(1000);
            garageDoorRelay.open();
            promptConfig();
            checkCommand();
            break;
        default:
            // Specified command is invalid.
            Serial.println(F("WARN: Unrecognized command."));
            promptConfig();
            checkCommand();
            break;
    }
}

/**
 * Enter fail-safe mode. This will suspend all tasks, disable relay activation,
 * and propmpt the user for configuration.
 */
void failSafe() {
    ESPCrashMonitor.defer();
    Serial.println();
    Serial.println(F("ERROR: Entering failsafe (config) mode..."));
    taskMan.disableAll();
    sensorLED.on();
    wifiLED.on();
    garageDoorRelay.open();
    promptConfig();
    checkCommand();
}

/**
 * Check for the user to press the 'i' key at the serial console to
 * interrupt normal operation and present the configuration menu.
 */
void checkInterrupt() {
    if (Serial.available() > 0 && Serial.read() == 'i') {
        failSafe();
    }
}

/**
 * Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to default SSID: "));
    Serial.print(DEFAULT_SSID);
    Serial.println(F("..."));
    
    connectWifi();
}

/**
 * Initializes the OTA update listener if enabled.
 */
void initOTA() {
    #ifdef ENABLE_OTA
        Serial.print(F("INIT: Starting OTA updater... "));
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.setPort(OTA_HOST_PORT);
            ArduinoOTA.setHostname(OTA_HOSTNAME);
            ArduinoOTA.setPassword(OTA_PASSWORD);
            ArduinoOTA.onStart([]() {
                // Handles start of OTA update. Determines update type.
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                }
                else {
                    type = "filesystem";
                }
                Serial.println("INFO: Starting OTA update (type: " + type + ") ...");
            });
            ArduinoOTA.onEnd([]() {
                // Handles update completion.
                Serial.println(F("INFO: OTA updater stopped."));
            });
            ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
                // Reports update progress.
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
 * Initializes the RS232 serial interface.
 */
void initSerial() {
    Serial.begin(SERIAL_BAUD);
    Serial.println();
    Serial.println();
    Serial.print(F("INIT: CyGarage v"));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F(" booting ..."));
    Serial.println();
}

/**
 * Initializes the required input pins.
 */
void initInputs() {
    Serial.print(F("INIT: Initializing sensors... "));
    openSensor.init();
    closeSensor.init();
    Serial.println(F("DONE"));
}

/**
 * Initializes output components.
 */
void initOutputs() {
    Serial.print(F("INIT: Initializing components... "));
    sensorLED.init();
    sensorLED.on();

    wifiLED.init();
    wifiLED.on();

    garageDoorRelay.init();
    garageDoorRelay.open();
    delay(1000);
    Serial.println(F("DONE"));
}

/**
 * Initializes the task manager and all recurring tasks.
 */
void initTaskManager() {
    Serial.print(F("INIT: Initializing task scheduler... "));

    taskMan.init();
    taskMan.addTask(tCheckActivation);
    taskMan.addTask(tCheckSensors);
    taskMan.addTask(tCheckWifi);
    
    tCheckWifi.enableDelayed(30000);
    tCheckSensors.enable();
    tCheckActivation.enable();
    Serial.println(F("DONE"));
}

/**
 * Callback routine for checking WiFi connectivity.
 */
void onCheckWiFi() {
    Serial.println(F("INFO: Checking WiFi connectivity..."));
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WARN: Lost connection. Attempting reconnect..."));
        connectWifi();
        if (WiFi.status() == WL_CONNECTED) {
            initMDNS();
            initWebServer();
            initOTA();
        }
    }
}

/**
 * Callback routine for checking if the door has been activated, and
 * deactivates it if it has.
 */
void onCheckActivation() {
    if (garageDoorRelay.isClosed()) {
        Serial.print(F("INFO: Deactivating door @ "));
        Serial.println(millis());
        garageDoorRelay.open();
    }
}

/**
 * Callback routine that checks the sensor inputs to determine door state.
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
}

void onRelayStateChange(RelayInfo* sender) {
    // TODO Currently nothing to do here yet. May not need this.
}

/**
 * Initializes the crash monitor and dump any previous crash data to the serial console.
 */
void initCrashMonitor() {
    Serial.print(F("INIT: Initializing crash monitor... "));
    ESPCrashMonitor.disableWatchdog();
    Serial.println(F("DONE"));
    ESPCrashMonitor.dump(Serial);
    delay(100);
}

/**
 * Bootstrap routine. Executes once at boot and initializes all subsystems in sequence.
 */
void setup() {
    initSerial();
    initCrashMonitor();
    initOutputs();
    initInputs();
    initWiFi();
    initMDNS();
    initWebServer();
    initOTA();
    initTaskManager();
    Serial.println(F("INIT: Boot sequence complete."));
    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
}

/**
 * Main loop. Executes all tasks that need to be, handles incoming web server requests,
 * and OTA update requests.
 */
void loop() {
    ESPCrashMonitor.iAmAlive();
    checkInterrupt();
    taskMan.execute();
    #ifdef ENABLE_MDNS
        mdns.update();
    #endif
    server.handleClient();
    #ifdef ENABLE_OTA
        ArduinoOTA.handle();
    #endif
}