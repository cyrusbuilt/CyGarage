# CyGarage

[![Build Status](https://travis-ci.com/cyrusbuilt/CyGarage.svg?branch=master)](https://travis-ci.com/cyrusbuilt/CyGarage)

Arduino-based (specifically the Adafruit Huzzah ESP8266) IoT garage door opener.

The code in this repository represents the Firmware for the CyGarage IoT garage door opener device, as well as some components for integrating with [OpenHAB2](https://openhab.org).  CyGarage is meant to integrate with an existing garage door opener and provide 2 things: The ability to detect the door's state (open, close, or ajar) and the ability to control the door over HTTP. Since there is an [ESP8266](https://www.adafruit.com/product/2471) at the heart of it, that means it is WiFi-enabled and is capable of OTA (Over-the-Air) firmware updates.

## Theory of Operation

The way this is intended to work is: when the device detects that the door is open or ajar, then user can see this status and (if integrated with OpenHAB or some other similar system) can get notifications when this occurs. When the device receives the activate command, it then triggers a relay which simulates the garage door button press which will in turn raise or lower the door depending on the current state.

## Configuration

In main.cpp, there is a configuration section. These options are the hardcoded default values that the system will initially boot with. However, if there is a config.json file present in the root of the SPIFFS file system, then it will override these defaults. Here we'll discuss each option:

- ENABLE_OTA: If you do not wish to support OTA updates, just comment this define.
- ENABLE_MDNS: If you do not wish to support [MDNS](https://tttapa.github.io/ESP8266/Chap08%20-%20mDNS.html), comment this define.
- DEFAULT_SSID: Change this line to reflect the SSID of your WiFi network.
- DEFAULT_PASSWORD: Change this line to reflect the password to your WiFi network.
- WEBSERVER_PORT: If you wish to use a port number other than the standard HTTP port, you can change that here.
- SERIAL_BAUD: While it is not recommended, you can change the BAUD rate of the serial port here.
- CHECK_WIFI_INTERVAL: The interval (in milliseconds) to check to make sure the device is still connected to WiFi, and if not attempt to reconnect. Default is 30 seconds.
- CHECK_SENSORS_INTERVAL: The interval (in millisceonds) to check the door contact sensors to determine door state. Default is 3 seconds.
- ACTIVATION_DURATION: The amount of time (in milliseconds) to activate the door relay. Default is 1 second and is recommended this value not be changed unless you have good reason for doing so.
- DEVICE_NAME: This essentially serves as the host name of the device on the network.
- OTA_HOST_PORT: Defines the port to listen for OTA updates on.
- OTA_PASSWORD: The password used to authenticate with the OTA server.
- ip: The default IP address. By default, this devices boots with a static IP configuration. The default IP is 192.168.0.200. You can change that here if you wish.
- gw: The default gateway address. The current default is 192.168.0.1. You can change that here if you wish.
- sm: The subnet mask. By default, it is 255.255.255.0, but you can change that here if need be.

To override the default configuration options, you need to upload a filesystem image containing a file named 'config.json' in the root of the SPIFFS filesystem. The file should like something link this:

```json
{
    "hostname": "CYGARAGE",
    "useDHCP": false,
    "ip": "your_device_ip_here",
    "gateway": "your_gateway_here",
    "subnetMask": "your_subnet_mask_here",
    "wifiSSID": "your_wifi_SSID_here",
    "wifiPassword": "your_wifi_password_here",
    "webserverPort": 80,
    "otaPort": 8266,
    "otaPassword": "your_OTA_password_here"
}
```

This configuration file is pretty self explanatory and one is included in the source. The file *MUST* be located in the "data" directory located in the root of the project in order to be picked up by the flash uploader (either via Serial or OTA). Each of the options in the configuration file are self-explanatory and match up with the hard-coded default settings mentioned above. If this file is not present when the firmware boots, a new file will be created and populated with the hardcoded defaults. These defaults can then be changed via the fail-safe menu and saved.

## Webserver Routes

- GET /    (Returns "Server ok..." in plain text)
- GET /version (Returns the firmware version in plain text)
- GET /device/status (Returns the device (door) status in plain text)
- POST /device/activate (Activates the relay and returns "OK" in plain text)

## OTA Updates

If you wish to be able to upload firmware updates Over-the-Air, then besides leaving the ENABLE_OTA option uncommented, you will also need to uncomment all the upload_* lines in platformio.ini, and change the line 'upload_port = ' line to reflect the IP of the device and the line '--auth=' to reflect whatever OTA_PASSWORD is set to. Then when you click "upload" from the PlatformIO tasks (or if you execute 'platformio run --target upload' from the command line) it should upload directly over WiFi and once completed, the device will automatically flash itself and reboot with the new version. If you wish to upload a new configuration file, you can also do this OTA. Assuming the above-mentioned settings are configured, you chan then click "Upload File System Image" from the PlatformIO project tasks.

## Serial Console Menu

If the device ever fails to connect to WiFi or if you press the 'I' key on your keyboard while in the serial console, normal operation of the device is suspended and the device will fall into a 'fail-safe' mode and present the user with a command menu. Here we will discuss those menu options:

- Reboot - press 'r' - Self-explanatory. Reboots the device.
- Configure network - press 'c'. Configure the network.

- - Press 'd' for DHCP or 't' for static IP.

- - If static, then you will also be prompted for the IP, gateway, and subnet mask.

- Scan wireless networks - Press 's'. This will scan for available wireless networks and dump a table of the networks it found to the console.

- Connect to new network - Press 'n'. This option prompts the user for a new SSID and password for different WiFi network to connect to. This option is currently experimental and doesn't seem to work that well.

- Reconnect to WiFi - Press 'w'. This can be used to reconnect to the current WiFi network if the connection was lost.

- Resume normal operation - Press 'e'. This will leave fail-safe mode and attempt to resume normal operation by resuming any suspended tasks and verify network connectivity.

- Get network info - Press 'g'. This will dump network information to the serial console (IP config, WiFi connection info).

- Activiate door - Press 'a'. This allows the user to manually activate the door.

- Save config changes - Press 'f'. This will save any configuration changes currently stored in memory to config.json.

- Restore default config - Press 'z'. This will restore the firmware configuration back to the factory default settings (the hard-coded defaults). It does this by simply deleting the existing config.json file from flash storage and then immediately rebooting the firmware. Upon reboot, a new config.json file is created and populated with the default settings which are then applied to the running configuration in memory.

## Dependencies

The firmware dependencies will be installed automatically at compile-time.  However, if you wish to install dependencies prior to compiling (for intellisense or to clear warnings/errors in the VSCode editor) you can run the following command:

```bash
platformio lib install
```

Then go to the platformio menu and click "Rebuild IntelliSense index".

## Tools

Included with this firmware is a few handy tools:

- /disassemble.sh - Disassembles the compiled firmware into a plain text file called 'disassembly.txt'. If the firmware has not been built, it will be compiled first, then disassembled. The disassembly output contains the assembly code, C++ calls, and instruction addresses useful for debugging the executing code or determining the cause of crashes at addresses reported by ESPCrashMonitor during boot.

- tools/clear_flash.sh - Completely clears the flash memory on the ESP8266 of all data (including firmware).

- tools/serial_monitor.sh - Just runs the serial monitor. Pretty much the same as clicking 'Monitor' in the platformio menu, but uses its own settings instead of those found in platformio.ini.

## Mobile App

If you choose not to integrate with OpenHAB, you can get similar functionality using the mobile app for iOS and Android located [here](https://github.com/cyrusbuilt/CyGarageMobile).
