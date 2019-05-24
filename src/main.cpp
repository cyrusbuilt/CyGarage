#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include "LED.h"
#include "Relay.h"
#include "TaskScheduler.h"
#include "ResetManager.h"

#define FIRMWARE_VERSION "1.0"

// Configuration
#define ENABLE_OTA                              // Comment this line to disable OTA updates.
#define DEFAULT_SSID "your_ssid_here"           // Put the SSID of your WiFi here.
#define DEFAULT_PASSWORD "your_password_here"   // Put your WiFi password here.
#define WEBSERVER_PORT 80                       // The built-in webserver port.
#define SERIAL_BAUD 115200                      // The BAUD rate (speed) of the serial port (console).
#define CHECK_WIFI_INTERVAL 30000               // How often to check WiFi status (milliseconds).
#define CHECK_SENSORS_INTERVAL 1000             // How often to check sensors (milliseconds).
#define ACTIVATION_DURATION 500                 // How long the activation relay should be on.
#define DEVICE_NAME "CyGarage"                  // The device name.
#ifdef ENABLE_OTA
    #define OTA_HOST_PORT 8266                     // The OTA updater port.
    #define OTA_HOSTNAME DEVICE_NAME               // The OTA updater host name. Should match device name.
    #define OTA_PASSWORD "your_ota_password_here"  // The OTA updater password.
#endif
IPAddress ip(192, 168, 0, 200);                 // The default static host IP.
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
MDNSResponder mdns;
ESP8266WebServer server(WEBSERVER_PORT);
Relay garageDoorRelay(PIN_ACTIVATE, onRelayStateChange, "GARAGE_DOOR");
LED sensorLED(PIN_SENSOR_LED, NULL);
LED wifiLED(PIN_WIFI_LED, NULL);
Task tCheckWifi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWiFi);
Task tCheckActivation(ACTIVATION_DURATION, TASK_FOREVER, &onCheckActivation);
Task tCheckSensors(CHECK_SENSORS_INTERVAL, TASK_FOREVER, &onCheckSensors);
Scheduler taskMan;
String ssid = DEFAULT_SSID;
String password = DEFAULT_PASSWORD;

/**
 * Scan for available networks and dump each discovered network to the console.
 */
void getAvailableNetworks() {
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
 * @param openSensorPin The pin being used for the open sensor.
 * @param closeSensorPin The pin being used for the close sensor.
 * @return The state string.
 */
String deviceStatus(uint8_t openSensorPin, uint8_t closeSensorPin) {
    String state = "AJAR";
    if (digitalRead(openSensorPin) == LOW) {
        state = "OPEN";
    }
    else if (digitalRead(closeSensorPin) == LOW) {
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
    server.send(200, "text/plain", deviceStatus(PIN_OPEN_SENSOR, PIN_CLOSE_SENSOR));
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
 * Initializes the MDNS responder.
 */
void initMDNS() {
    Serial.print(F("INIT: Starting MDNS responder"));
    if (WiFi.status() == WL_CONNECTED) {
        while (!mdns.begin(DEVICE_NAME)) {
            delay(1000);
            Serial.print(F("."));
        }

        Serial.println(F(" DONE"));
    }
    else {
        Serial.println(F(" FAILED"));
    }
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
    Serial.println(F("=                            ="));
    Serial.println(F("=============================="));
    Serial.println();
    Serial.println("Enter command choice (r/c/s/w): ");
    while (Serial.available() < 1) {
        delay(50);
    }
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
 * Attempt to connect to 
 */
void connectWifi() {
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
    
    int maxTries = 3;
    int currentTry = 0;
    while ((WiFi.status() != WL_CONNECTED) && (currentTry < maxTries)) {
        currentTry++;
        wifiLED.blink(500);
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("ERROR: Failed to connect to WiFi!"));
        failSafe();
    }
    else {
        Serial.println(F("INFO: WiFi connected."));
        Serial.print(F("INFO: Local IP address: "));
        Serial.println(WiFi.localIP());
        Serial.print(F("INFO: MAC address: "));
        Serial.println(WiFi.macAddress());
    }
}

/**
 * Checks commands entered by the user via serial input and carries out
 * the specified action if valid.
 */
void checkCommand() {
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
            Serial.println(F("Choose network mode (d = DHCP, t = Static"));
            while (Serial.available() < 1) {
                delay(50);
            }
            checkCommand();
            break;
        case 'd':
            // Switch to DHCP mode.
            ESP.eraseConfig();
            WiFi.mode(WIFI_STA);
            promptConfig();
            checkCommand();
            break;
        case 't':
            // Switch to static IP mode. Request IP settings.
            ESP.eraseConfig();
            Serial.println(F("Enter IP address: "));
            while (Serial.available() < 1) {
                delay(50);
            }
            str = Serial.readString();
            ip = getIPFromString(str);
            Serial.println(F("Enter gateway: "));
            while (Serial.available() < 1) {
                delay(50);
            }
            str = Serial.readString();
            gw = getIPFromString(str);
            Serial.println(F("Enter subnet mask: "));
            while (Serial.available() < 1) {
                delay(50);
            }
            str = Serial.readString();
            sm = getIPFromString(str);
            WiFi.config(ip, gw, sm);
            promptConfig();
            checkCommand();
            break;
        case 'w':
            // Attempt to reconnect to WiFi.
            onCheckWiFi();
            if (WiFi.status() == WL_CONNECTED) {
                resumeNormal();
            }
            else {
                Serial.println(F("ERROR: Still no network connection."));
                promptConfig();
                checkCommand();
            }
            break;
        case 'n':
            Serial.println(F("Enter new SSID: "));
            while (Serial.available() < 1) {
                delay(50);
            }

            ssid = Serial.readString();
            Serial.println(F("Enter new password: "));
            while(Serial.available() < 1) {
                delay(50);
            }

            password = Serial.readString();
            connectWifi();
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
 * Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to default SSID: "));
    Serial.print(DEFAULT_SSID);
    Serial.println(F("..."));
    
    WiFi.config(ip, gw, sm);
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
    pinMode(PIN_CLOSE_SENSOR, INPUT);
    pinMode(PIN_OPEN_SENSOR, INPUT);
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
        WiFi.begin(ssid, password);
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
    if (digitalRead(PIN_ACTIVATE) == HIGH) {
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
    if (deviceStatus(PIN_OPEN_SENSOR, PIN_CLOSE_SENSOR) == "AJAR") {
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
 * Bootstrap routine. Executes once at boot and initializes all subsystems in sequence.
 */
void setup() {
    initSerial();
    initOutputs();
    initInputs();
    initWiFi();
    initMDNS();
    initWebServer();
    initOTA();
    initTaskManager();
    Serial.println(F("INIT: Boot sequence complete."));
}

/**
 * Main loop. Executes all tasks that need to be, handles incoming web server requests,
 * and OTA update requests.
 */
void loop() {
    taskMan.execute();
    server.handleClient();
    ArduinoOTA.handle();
}