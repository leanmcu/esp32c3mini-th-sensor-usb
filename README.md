# Esp32C3 mini Temperature and Humidity Sensor USB


## Description:
This is a temperature and humidity sensor you can <b>build</b> in approximately <b>1 hour</b>, with <b>off the shelf components</b> that will costs about <b>$7</b> from AliExpress (under $15 from amazon.com).

This repository contains the complete solution for PlatformIO and the 3d printable STL files for the enclosure. The enclosure files are located in the enclosure subdirectory.

The assembly requires only 4 wires:
- HTU21D sensor connected to I2C (SDA -> GPIO8, SCL -> GPIO9)
- HTU21D VCC -> ESP32 3.3V
- HTU21D GND -> ESP32 GND


## Installation

Connect the Esp32C3 board to your computer, and open the following link in Chrome or Edge browser. Then click the "CONNECT & FLASH" button.

https://leanmcu.github.io/esp32c3mini-th-sensor-usb/

## Configuration

The code allows you to send data to <b>Home Assistant</b> and/or <b>ThingSpeak</b>. For Home Assistant, the code performs automatic registration using MQTT auto discovery, so you don't have to write/modify any YML files.

If you don't have a Home Assistant server or prefer a cloud based solution, you can create a free account on ThingSpeak.com that will work very well for up to 4 temperature and humidity sensors. You can view the ThingSpeak data either from a browser or from the ThingView mobile app.

In order to enter the configuration mode, press on the Esp32C3 Reset button, release it, then press on Boot button and hold it for at least a second before releasing. The blue led will flash and an access point is created. Connect to that access point and in your browser point to http://192.168.4.1


// mandatory settings
  - HOME_ASSISTANT true if you want to use Home Assistant auto-discovery, false otherwise
  - THINGSPEAK true if you want to send data to ThingSpeak, false otherwise
// WiFi settings, mandatory
  - WiFi ssid
  - WiFi password
// MQTT settings, mandatory if HOME_ASSISTANT is true
  - mqtt_server and mqtt_server_ip; by default, mqtt_server is used, but you can also input the IP address of the MQTT server to avoid DNS resolution
  - mqtt_user
  - mqtt_pass
// ThingSpeak settings, mandatory if THINGSPEAK is true
  - hingSpeakChannelNumber
  - thingSpeakWriteAPIKey
// reporting period, mandatory
  - timerDelay: deep sleep time in seconds (minimum 15, recommended 180 or more to avoid sensor overheating from esp32 heat)

## Libs 
For extra convenience, the solution also includes the following libraries:</br>


HTU21D-Sensor-Library
  [https://github.com/devxplained/HTU21D-Sensor-Library] </br>
pubsubclient
  [https://github.com/knolleary/pubsubclient]
WiFiManager
  [https://github.com/tzapu/wifimanager]

## License

This source code is licensed under the terms of the GNU General Public
License version 2 only (GPL-2.0-only), as published by the Free Software
Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License version 2 for more details.

You should have received a copy of the GNU General Public License
version 2 along with this program. If not, see:
   https://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 
