#ifndef _CONSOLE_H
#define _CONSOLE_H

#include <Arduino.h>
#include <IPAddress.h>

class ConsoleClass
{
public:
    ConsoleClass();
    IPAddress getIPFromString(String value);
    void waitForUserInput();
    String getInputString(bool isPassword = false);
    void enterCommandInterpreter();
    void onRebootCommand(void (*rebootHandler)());
    void onScanNetworks(void (*scanHandler)());
    void setHostName(String hostName);
    void setMqttConfig(String broker, int port, String username, String password, String conChan, String statChan);
    void onHostNameChange(void (*hostNameChangeHandler)(const char* newHostName));
    void onDhcpConfig(void (*dhcpHandler)());
    void onStaticConfig(void (*staticHandler)(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns));
    void onReconnectCommand(void (*reconnectHandler)());
    void onWiFiConfigCommand(void (*wifiConfigHandler)(String newSsid, String newPassword));
    void onResumeCommand(void (*resumeHandler)());
    void onGetNetInfoCommand(void (*netInfoHander)());
    void onSaveConfigCommand(void (*saveConfigHandler)());
    void onMqttConfigCommand(void (*mqttChangeHandler)(String newBroker, int newPort, String newUsername, String newPass, String newConChan, String newStatChan));
    void onConsoleInterrupt(void (*interruptHandler)());
    void onActivateDoor1(void (*activateDoor1Handler)());
    void onFactoryRestore(void (*factoryRestoreHandler)());
    #ifdef CG_MODEL_2
    void onActivateDoor2(void (*activateDoor2Handler)());
    #endif
    void checkInterrupt();

private:
    void displayMenu();
    void checkCommand();
    void configureStaticIP();
    void configureWiFiNetwork();
    void configMQTT();

    void (*rebootHandler)();
    void (*scanHandler)();
    void (*hostNameChangeHandler)(const char* newHostName);
    void (*dhcpHandler)();
    void (*staticHandler)(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns);
    void (*reconnectHandler)();
    void (*wifiConfigHandler)(String newSsid, String newPassword);
    void (*resumeHandler)();
    void (*netInfoHandler)();
    void (*saveConfigHandler)();
    void (*mqttChangeHandler)(String newBroker, int newPort, String newUsername, String newPass, String newConChan, String newStatChan);
    void (*interruptHandler)();
    void (*activateDoor1Handler)();
    void (*factoryRestoreHandler)();
    #ifdef CG_MODEL_2
    void (*activateDoor2Handler)();
    #endif
    String _hostName;
    String _mqttBroker;
    int _mqttPort;
    String _mqttUsername;
    String _mqttPassword;
    String _mqttControlChannel;
    String _mqttStatusChannel;
};

extern ConsoleClass Console;
#endif