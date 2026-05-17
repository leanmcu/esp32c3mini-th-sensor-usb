/*
 * SPDX-License-Identifier: GPL-2.0-only OR LicenseRef-Commercial
 *
 * Copyright (c) 2026 LeanMCU
 *
 *
 * Dual Licensing Notice:
 *
 * This source code is licensed under the terms of the GNU General Public
 * License version 2 only (GPL-2.0-only), as published by the Free Software
 * Foundation.
 *
 * Alternatively, this software may be licensed under a separate commercial
 * license obtained from the copyright holder. Use of this software in
 * proprietary or closed-source products, or under terms incompatible with
 * the GPL-2.0-only, requires a valid commercial license.
 *
 * Commercial licensing inquiries:
 *   leanmcu(at)gmail(dot)com
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License version 2 for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 along with this program. If not, see:
 *   https://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

/*
Hardware connections:
- HTU21D sensor connected to I2C (SDA = GPIO8, SCL = GPIO9)
- HTU21D VCC to ESP32 3.3V
- HTU21D GND to ESP32 GND
- ESP32-C3 pro mini module powered via USB
- No other connections are needed
*/

#include <Arduino.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <Ticker.h>
#include <HTU21D.h>

#define STATE_TOPIC_SIZE 64
#define DISCOVERY_TOPIC_SIZE 128
#define PAYLOAD_SIZE 2048
const unsigned long mqttRetryInterval = 5000;  // 5s
const unsigned long wifiRetryInterval = 10000; // 10s
const uint8_t sendRetries = 5;                 // number of retries for ThingSpeak write
const int BOOT_BUTTON_PIN = 9;
bool _isConfigured = false;
// web exposed settings start
bool _isHomeAssistant = true;
bool _isThingSpeak = true;
bool _isFahrenheit = false;
String _ssid;
String _password;
String _mqttServer;
String _mqttUser;
String _mqttPass;
String _tsChannel;
String _tsKey;
uint8_t _tsFieldTemp;
uint8_t _tsFieldHum;
uint8_t _tsFieldRssi;
uint32_t _timerDelay;
float _tempOffset;
float _humOffset;
float _emaAlpha;

Preferences _prefs; // web exposed settings end here
char _stateTopic[STATE_TOPIC_SIZE];
char _discoveryTopic[DISCOVERY_TOPIC_SIZE];
char _discoveryPayload[PAYLOAD_SIZE];
char _macAddr[6];
char _macAddrStr[13];
unsigned long _lastMqttAttempt = 0;
unsigned long _lastWifiCheck = 0;
float _temperature;
float _humidity;
unsigned long _startingMillis;
// Variables stored in RTC memory survive Deep Sleep
RTC_DATA_ATTR uint8_t _bootCount;
RTC_DATA_ATTR bool _isRegisteredInHA;
RTC_DATA_ATTR float _emaTemperature = -999.0;
RTC_DATA_ATTR float _emaHumidity = -999.0;
int8_t _rssi;
WiFiClient _wifiClient;
WiFiClient _tsClient;
PubSubClient _mqttClient(_wifiClient);
HTU21D _htu21d; // HTU21D sensor instance
volatile bool _mqttEchoReceived = false;
String _mqttEchoPayload;

void MqttConnect(void);
void GetMacAddress(void);
void HtuCreateDiscoveryTopic(char *macAddrStr, char *jsonTopic);
void HtuCreateStateTopic(char *macAddrStr, char *stateTopic);
void HtuCreateDiscoveryPayload(char *macAddrStr, char *jsonPayload);
void HtuCreateStatePayload(char *statePayload);
void SetupSensor(void);
void ReadSensor(void);
bool ReadHtu21(float *temperature, float *compHumidity);
void ParamsSetup(void);
void LoadSettings(void);
void SendDataToHA(void);
void SendDataToThingSpeak(void);
bool ConnectToWiFi(void);
bool WaitForNetwork(void);
void Sleep(void);
bool IsNumeric(String s, bool isPositive = false);
bool IsPositiveInteger(String s);
void NonBlockingDelay(unsigned long ms);
void ToggleLED(void);
void MqttCallback(char *topic, byte *payload, unsigned int length);

const char *_custom_css = R"rawliteral(
<style>
  /* 1. Base Colors */
  body { background-color: #121212 !important; color: #e0e0e0 !important; }
  .wrap { background-color: #1e1e1e !important; padding: 20px; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
  
  /* 2. Logo Style */
  #logo { font-family: 'Arial Rounded MT Bold', sans-serif; color: #39ff14; font-size: 2.2em; font-weight: bold; margin-bottom: 20px; text-shadow: 0 0 8px #39ff14; text-align: center; }

  input:invalid { border: 2px solid red; }
  input:valid { border: 2px solid green; }
  /* 3. SSID Links & Icons */
  a { color: #39ff14 !important; } /* Neon Green Links */
  a:hover { color: #03dac6 !important; }
  .q { filter: invert(1) hue-rotate(80deg) brightness(1.5); } /* Makes the WiFi bars neon green */

  /* 4. Form Inputs */
  input, select { background-color: #333 !important; color: #fff !important; border: 1px solid #444 !important; }
  label { color: #bb86fc !important; } /* Purple labels for contrast */
  hr { border-color: #444 !important; }

  /* 5. Buttons - Neon Green */
  button, input[type='submit'] { 
    background-color: #39ff14 !important; 
    color: #000 !important; 
    font-weight: bold !important;
    text-transform: uppercase;
  }
  button:hover { background-color: #32cd32 !important; }

  /* 6. Messages */
  .msg { background-color: #222 !important; border-left-color: #39ff14 !important; color: #fff !important; }
</style>
)rawliteral";

void setup()
{
  NonBlockingDelay(500); // Allow time for serial monitor to connect
  _startingMillis = millis();
  Serial.begin(115200);
  ParamsSetup();

  if (_bootCount == 0)
  {
    SetupSensor();
  }
  else
  {
    Wire.begin();
  }
  ReadSensor();

  if (!ConnectToWiFi())
  {
    Serial.println("Failed to connect to WiFi. Sleeping...");
    Sleep();
  }
  else
  {
    if (!WaitForNetwork())
    {
      Serial.println("Network is not available. Sleeping...");
      Sleep();
    }
  }

  NonBlockingDelay(1000);
  _rssi = WiFi.RSSI();
  yield();
  delay(200);
  GetMacAddress();

  if (_isHomeAssistant)
  {
    Serial.println("Send data to home assistant...\n");
    SendDataToHA();
    Serial.println("Done...\n");
  }

  if (_isThingSpeak)
  {
    Serial.println("Send data to ThingSpeak...\n");
    SendDataToThingSpeak();
    Serial.println("Done...\n");
  }

  if (_bootCount == 0)
  {
    _bootCount = 1;
  }
  Sleep();
}

void loop()
{
}

void MqttConnect(void)
{
  if (_mqttClient.connected())
    return;

  _wifiClient.setTimeout(5000);
  // resolve .local if necessary
  if (_mqttServer.endsWith(".local"))
  {
    Serial.print("Resolving mDNS: ");
    Serial.println(_mqttServer);
    String hostStr = _mqttServer.substring(0, _mqttServer.length() - 6);
    if (!MDNS.begin("lean-sensor"))
    {
      Serial.println("Error setting up MDNS responder!");
    }
    // Attempt to find the IP of the .local host
    IPAddress mqttIP = MDNS.queryHost(hostStr.c_str());
    if (mqttIP != INADDR_NONE)
    {
      Serial.print("Resolved to: ");
      Serial.println(mqttIP);
      _mqttClient.setServer(mqttIP, 1883); // Point client to the actual IP
    }
    else
    {
      Serial.println("mDNS Failed to resolve.");
      return; // Exit early to prevent "infinite" retry loops on a dead address
    }
  }
  else
  {
    // If it's just "homeassistant" or an IP address,
    // the PubSubClient handles it automatically via standard DNS
    _mqttClient.setServer(_mqttServer.c_str(), 1883);
  }
  int retry = 0;
  while (!_mqttClient.connected() && retry < sendRetries)
  {

    Serial.printf("Attempting MQTT connection with %s\n", _mqttServer.c_str());
    _mqttClient.disconnect();           // Ensure any previous connection is fully closed
    _wifiClient.stop();                 // Kill previous TCP socket
    delay(100);                         // Short delay to ensure socket is fully closed
    _mqttClient.setClient(_wifiClient); // Rebind to a fresh client

    // The 'true' at the end forces a Clean Session, telling the broker to
    // immediately discard any old "ghost" connections for this client ID.
    if (_mqttClient.connect(_macAddrStr, _mqttUser.c_str(), _mqttPass.c_str(), nullptr, 0, 0, nullptr, true))
    {
      Serial.println("MQTT connected");
      // Give the Broker 200ms to finish the handshake before we start subscribing/publishing
      unsigned long start = millis();
      while (millis() - start < 200)
      {
        _mqttClient.loop();
        delay(10);
      }
      return;
    }
    else
    {
      Serial.printf("MQTT connection failed, rc=%d. Retrying in %lu ms...\n", _mqttClient.state(), mqttRetryInterval);
    }
    unsigned long pause = millis();
    while (millis() - pause < 2000)
    {
      yield(); // Keep WiFi stack alive during the wait
      delay(10);
    }
    retry++;
  }
}

void HtuCreateStateTopic(char *macAddrStr, char *stateTopic)
{
  snprintf(stateTopic, STATE_TOPIC_SIZE, "sensor%s/state", macAddrStr);
}

void HtuCreateDiscoveryTopic(char *macAddrStr, char *jsonTopic)
{
  snprintf(jsonTopic, DISCOVERY_TOPIC_SIZE, "homeassistant/device/%s/config", macAddrStr);
}

void HtuCreateDiscoveryPayload(char *macAddrStr, char *jsonPayload)
{
  char payload[] = "{\
  \"dev\": {\
    \"ids\": \"th_ABCDEFGHIJKL\",\
    \"name\": \"TH-sensor_IIII\",\
    \"mf\": \"LeanMCU\",\
    \"mdl\": \"TH\",\
    \"sw\": \"1.0\",\
    \"sn\": \"ABCDEFGHIJKL\",\
    \"hw\": \"1.0\"\
  },\
  \"o\": {\
    \"name\":\"TH sensor origin\",\
    \"sw\": \"1.0\",\
    \"url\": \"https://bla2mqtt.example.com/support\"\
  },\
  \"cmps\": {\
    \"temp\": {\
      \"p\": \"sensor\",\
      \"device_class\":\"temperature\",\
      \"unit_of_measurement\":\"°C\",\
      \"value_template\":\"{{ value_json.temp}}\",\
      \"unique_id\":\"ABCDEFGHIJKLt\"\
    },\
    \"hum\": {\
      \"p\": \"sensor\",\
      \"device_class\":\"humidity\",\
      \"unit_of_measurement\":\"%\",\
      \"value_template\":\"{{ value_json.hum}}\",\
      \"unique_id\":\"ABCDEFGHIJKLh\"\
    },\
     \"rssi\": {\
      \"p\": \"sensor\",\
      \"device_class\":\"signal_strength\",\
      \"unit_of_measurement\":\"dBm\",\
      \"value_template\":\"{{ value_json.rssi}}\",\
      \"unique_id\":\"ABCDEFGHIJKLr\"\
    }\
  },\
  \"state_topic\":\"sensorABCDEFGHIJKL/state\"\
}";

  memcpy(jsonPayload, payload, strlen(payload) + 1);
  char *index = strstr(jsonPayload, "IIII");
  memcpy(index, macAddrStr + 8, 4); // Use last 4 chars of MAC for device name

  index = strstr(jsonPayload, "ABCDEFGHIJKL");
  while (index != NULL)
  {
    memcpy(index, macAddrStr, 12);
    index = strstr(index + 12, "ABCDEFGHIJKL");
  }
}

void GetMacAddress(void)
{
  uint8_t baseMac[6];
  WiFi.macAddress(baseMac);
  memcpy(_macAddr, baseMac, 6);
  sprintf(_macAddrStr, "%02X%02X%02X%02X%02X%02X",
          _macAddr[0], _macAddr[1], _macAddr[2],
          _macAddr[3], _macAddr[4], _macAddr[5]);
}

void SetupSensor(void)
{
  _htu21d.begin(); // Initialize the HTU21D sensor
}

float calculateEMA(float current, float previous, float alpha)
{
  // If this is the first run after a hard reset, just return current
  if (previous < -100.0)
    return current;

  // EMA Formula
  return (alpha * current) + ((1.0 - alpha) * previous);
}

void ReadSensor(void)
{
  uint8_t dataPointNo = 0;
  float temp[3], hum[3];
  while (dataPointNo < 3)
  {
    if (ReadHtu21(&temp[dataPointNo], &hum[dataPointNo]))
    {
      dataPointNo++;
      unsigned long s = millis();
      while (millis() - s < 50)
      {
        yield();
      }
    }
  }
  std::sort(temp, temp + 3);
  std::sort(hum, hum + 3);
  if (_isFahrenheit)
  {
    temp[1] = temp[1] * 9.0 / 5.0 + 32.0; // Convert to Fahrenheit
  }
  temp[1] += _tempOffset;
  hum[1] += _humOffset;
  if (_bootCount == 0) // If this is the first boot, initialize EMA with the first reading
  {
    _emaTemperature = temp[1];
    _emaHumidity = hum[1];
  }
  else
  {
    // For subsequent boots, calculate EMA
    _emaTemperature = calculateEMA(temp[1], _emaTemperature, _emaAlpha);
    _emaHumidity = calculateEMA(hum[1], _emaHumidity, _emaAlpha);
  }
  // Use the filtered values for transmission
  _temperature = _emaTemperature;
  _humidity = _emaHumidity;
}

bool ReadHtu21(float *temperature, float *compHumidity)
{
  bool status;
  float compHum;
  float temp;
  if (_htu21d.measure() == true)
  {
    temp = _htu21d.getTemperature();
    compHum = _htu21d.getHumidity();
    status = true;
    *temperature = temp;
    *compHumidity = compHum;
  }
  else
  {
    status = false;
  }
  return status;
}

void HtuCreateStatePayload(char *statePayload)
{
  char payload[128];
  snprintf(payload, sizeof(payload), "{\"temp\":%.2f,\"hum\":%.2f,\"rssi\":%d}", _temperature, _humidity, _rssi);
  memcpy(statePayload, payload, strlen(payload) + 1);
}

void ToggleLED()
{
  digitalWrite(8, !digitalRead(8));
}

void ParamsSetup(void)
{
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  delay(500); // Short delay for pull-up to settle

  WiFiManager wm;
  wm.setConnectTimeout(30);
  wm.setConfigPortalTimeout(180);

  bool isConfigRequested = false;
  LoadSettings(); // Load settings to populate the portal with current values

  int count = 0;
  // We only wait for the button if the ESP32 was NOT woken up by the timer
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER)
  {
    while (digitalRead(BOOT_BUTTON_PIN) == HIGH && count < 300)
    {
      delay(10);
      count++;
    }
    if (count < 300) // If button was pressed within 3 seconds from restart
    {
      count = 0;
      while (digitalRead(BOOT_BUTTON_PIN) == LOW && count < 150) // if button was pressed for at least 1 second
      {
        delay(10);
        count++;
      }
      if (count > 100) // If button is held for more than 1 second, reset settings
      {
        isConfigRequested = true;
        Serial.println("Settings Wiped. Entering Portal...");
      }
    }
  }

  // if configuration is not done or requested
  if (!_isConfigured || isConfigRequested)
  {
    pinMode(8, OUTPUT);
    Ticker ledTicker;
    ledTicker.attach(0.5, ToggleLED); // Toggle LED every 500ms
    Serial.println("Device not configured or config requested. Starting Portal...");
    Serial.println(_isFahrenheit);
    wm.setCustomHeadElement(_custom_css);
    wm.setSaveConfigCallback([]()
                             { Serial.println("Should save config"); });

    String ha_attr = "type='checkbox' " + String(_isHomeAssistant ? "checked" : "");
    String ts_attr = "type='checkbox' " + String(_isThingSpeak ? "checked" : "");
    WiFiManagerParameter custom_delay("delay", "Measurement period (seconds)", String(_timerDelay).c_str(), 10, "type='number' min='30' max='3600' step='1' required");
    WiFiManagerParameter custom_unit_hidden("unit", "", _isFahrenheit ? "F" : "C", 2);

    const char *unit_radio_c = "<style>#unit{display:none;}</style>"
                               "<br><label for='unit'>Temperature Unit</label><br>"
                               "<input type='radio' name='unit' value='C' checked "
                               "> Celsius<br>"
                               "<input type='radio' name='unit' value='F' "
                               "> Fahrenheit<br>";

    const char *unit_radio_f = "<style>#unit{display:none;}</style>"
                               "<br><label for='unit_pref'>Temperature Unit</label><br>"
                               "<input type='radio' name='unit' value='C' "
                               "> Celsius<br>"
                               "<input type='radio' name='unit' value='F' checked "
                               "> Fahrenheit<br>";

    const char *selected_radio_html = _isFahrenheit ? unit_radio_f : unit_radio_c;

    WiFiManagerParameter custom_unit_pref(selected_radio_html);
    WiFiManagerParameter custom_ha("ha", "Use Home Assistant", "T", 2, ha_attr.c_str(), WFM_LABEL_AFTER);
    WiFiManagerParameter custom_mqtt_serv("serv", "MQTT Server", _mqttServer.c_str(), 40);
    WiFiManagerParameter custom_mqtt_user("user", "MQTT User", _mqttUser.c_str(), 20);
    WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", _mqttPass.c_str(), 20, "type='password'");
    WiFiManagerParameter custom_ts("ts", "Use ThingSpeak", "T", 2, ts_attr.c_str(), WFM_LABEL_AFTER);
    WiFiManagerParameter custom_ts_key("key", "ThingSpeak API Key", _tsKey.c_str(), 20);
    WiFiManagerParameter custom_tempOffset("temp_offset", "Temperature Calibration Offset (°C/°F)", String(_tempOffset).c_str(), 4, "type='number' step='0.1' min='-10' max='10' required");
    WiFiManagerParameter custom_humOffset("hum_offset", "Humidity Calibration Offset (%)", String(_humOffset).c_str(), 4, "type='number' step='0.1' min='-10' max='10' required");
    WiFiManagerParameter custom_emaAlpha("ema_alpha", "EMA Smoothing Factor (0.0-1.0)", String(_emaAlpha).c_str(), 4, "type='number' step='0.1' min='0.1' max='1.0' required");
    WiFiManagerParameter custom_hr1("<hr>");
    WiFiManagerParameter custom_hr2("<hr>");
    WiFiManagerParameter custom_br1("<br/>");
    WiFiManagerParameter custom_br2("<br/>");
    WiFiManagerParameter autofill_absorber("<input style='display:none' type='text' name='user_name_v1'><input style='display:none' type='password' name='password_v1'>");

    wm.addParameter(&autofill_absorber);
    wm.addParameter(&custom_unit_pref);
    wm.addParameter(&custom_unit_hidden);
    wm.addParameter(&custom_delay);
    wm.addParameter(&custom_tempOffset);
    wm.addParameter(&custom_humOffset);
    wm.addParameter(&custom_emaAlpha);
    wm.addParameter(&custom_hr1);
    wm.addParameter(&custom_ha);
    wm.addParameter(&custom_br1);
    wm.addParameter(&custom_mqtt_serv);
    wm.addParameter(&custom_mqtt_user);
    wm.addParameter(&custom_mqtt_pass);
    wm.addParameter(&custom_hr2);
    wm.addParameter(&custom_ts);
    wm.addParameter(&custom_br2);
    wm.addParameter(&custom_ts_key);

    if (!wm.startConfigPortal("LeanMCU HomeNode"))
    {
      Serial.println("Portal Timeout/Error");
      ledTicker.detach(); // Stop LED ticker before restart
      for (int i = 0; i < 5; i++)
      {
        digitalWrite(8, LOW);
        delay(100);
        digitalWrite(8, HIGH);
        delay(100);
      }
      ESP.restart();
    }

    // save the custom parameters to Preferences
    _prefs.begin("settings", false);
    _prefs.putBool("configured", true);
    // Checkboxes: if checked, browser sends value ("T"), if not, it sends nothing/null
    _prefs.putBool("use_ha", (String(custom_ha.getValue()) == "T"));
    _prefs.putBool("use_ts", (String(custom_ts.getValue()) == "T"));

    _prefs.putString("unit_pref", String(custom_unit_hidden.getValue()));
    Serial.printf("unit_pref %s\n", String(custom_unit_hidden.getValue()).c_str());

    _prefs.putString("mqtt_serv", custom_mqtt_serv.getValue());
    _prefs.putString("mqtt_user", custom_mqtt_user.getValue());
    _prefs.putString("mqtt_pass", custom_mqtt_pass.getValue());
    _prefs.putString("ts_key", custom_ts_key.getValue());

    String timerDelay = custom_delay.getValue();
    if (IsPositiveInteger(timerDelay))
    {
      _prefs.putString("delay", custom_delay.getValue());
    }

    String tempOffset = custom_tempOffset.getValue();
    if (IsNumeric(tempOffset))
    {
      _prefs.putString("temp_offset", custom_tempOffset.getValue());
    }

    String humOffset = custom_humOffset.getValue();
    if (IsNumeric(humOffset))
    {
      _prefs.putString("hum_offset", custom_humOffset.getValue());
    }

    String emaAlpha = custom_emaAlpha.getValue();
    if (IsNumeric(emaAlpha))
    {
      _prefs.putString("ema_alpha", custom_emaAlpha.getValue());
    }

    Serial.println(custom_mqtt_serv.getValue());
    Serial.println("Configuration saved! Restarting...");

    if (WiFi.status() == WL_CONNECTED)
    {
      uint8_t attempts = 0;
      _prefs.putString("ssid", WiFi.SSID());
      _prefs.putString("pass", WiFi.psk());
      while ((WiFi.localIP() == INADDR_NONE) && attempts < 100)
      {
        delay(100);
        attempts++;
      }
    }
    _prefs.end();

    ledTicker.detach();    // Stop LED ticker before restart
    wm.stopConfigPortal(); // Ensure portal is stopped before restart
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
    delay(2000);
    digitalWrite(8, HIGH);
    ESP.restart();
  }
}

void LoadSettings()
{
  _prefs.begin("settings", false);
  // Load existing values to show as defaults in the portal
  _ssid = _prefs.getString("ssid", "");
  _password = _prefs.getString("pass", "");
  _isConfigured = _prefs.getBool("configured", false);
  _isHomeAssistant = _prefs.getBool("use_ha", true);
  _isThingSpeak = _prefs.getBool("use_ts", false);
  _isFahrenheit = (_prefs.getString("unit_pref", "C") == "F");
  _mqttServer = _prefs.getString("mqtt_serv", "homeassistant.local");
  _mqttUser = _prefs.getString("mqtt_user", "");
  _mqttPass = _prefs.getString("mqtt_pass", "");
  _tsKey = _prefs.getString("ts_key", "");
  _timerDelay = _prefs.getString("delay", "180").toInt();
  _tempOffset = _prefs.getString("temp_offset", "0.0").toFloat();
  _humOffset = _prefs.getString("hum_offset", "0.0").toFloat();
  _emaAlpha = _prefs.getString("ema_alpha", "0.2").toFloat();
  _prefs.end();
}

void MqttCallback(char *topic, byte *payload, unsigned int length)
{
  _mqttEchoPayload = "";

  for (unsigned int i = 0; i < length; i++)
  {
    _mqttEchoPayload += (char)payload[i];
  }
  _mqttEchoReceived = true;
}

void SendDataToHA(void)
{
  uint8_t mqttRetries = 0;
  bool publishResult = false;

  _mqttClient.setServer(_mqttServer.c_str(), 1883);
  _mqttClient.setKeepAlive(15);
  _mqttClient.setBufferSize(PAYLOAD_SIZE);
  _mqttClient.setCallback(MqttCallback);

  MqttConnect();

  if (!_mqttClient.connected())
  {
    Serial.println("MQTT NOT connected.");
    return;
  }

  if (!_isRegisteredInHA)
  {
    HtuCreateDiscoveryTopic(_macAddrStr, _discoveryTopic);
    HtuCreateDiscoveryPayload(_macAddrStr, _discoveryPayload);

    bool responseMqtt = false;
    if (_mqttClient.connected())
    {
      while (responseMqtt == false && mqttRetries < sendRetries)
      {
        responseMqtt = _mqttClient.publish(_discoveryTopic, _discoveryPayload, true);
        if (responseMqtt)
        {
          Serial.println("Discovery published successfully.");
          _isRegisteredInHA = true;
        }
        Serial.print("Discovery publish result: ");
        Serial.println(responseMqtt);
        Serial.println("mqttRetries: " + String(mqttRetries));
        mqttRetries++;
        unsigned long startWait = millis();
        while (millis() - startWait < 1000) // 1000ms is plenty for discovery
        {
          _mqttClient.loop();
          yield();
        }
      }
    }
  }

  char statePayload[128];
  HtuCreateStateTopic(_macAddrStr, _stateTopic);
  HtuCreateStatePayload(statePayload);

  Serial.println("Subscribing for echo...");
  _mqttClient.subscribe(_stateTopic);

  mqttRetries = 0;
  while (!publishResult && mqttRetries < sendRetries)
  {
    _mqttEchoReceived = false;
    _mqttEchoPayload = "";

    Serial.println("Publishing state...");
    publishResult = _mqttClient.publish(_stateTopic, statePayload);

    Serial.print("Publish result: ");
    Serial.println(publishResult);

    if (!publishResult)
    {
      mqttRetries++;
      _mqttClient.loop();
      delay(200);
      continue;
    }

    //  Wait for echo
    unsigned long startWait = millis();

    while (!_mqttEchoReceived && millis() - startWait < 5000)
    {
      _mqttClient.loop();
      delay(10);
      yield();
    }

    // validate payload
    if (_mqttEchoReceived)
    {
      if (_mqttEchoPayload == statePayload)
      {
        Serial.println("MQTT delivery CONFIRMED");
        break;
      }
      else
      {
        Serial.println("Echo received but payload mismatch:");
        Serial.println("Expected: " + String(statePayload));
        Serial.println("Received: " + _mqttEchoPayload);
      }
    }
    else
    {
      Serial.println("Echo NOT received.");
    }
    publishResult = false;
    mqttRetries++;
  }

  if (!publishResult)
  {
    Serial.println("MQTT delivery FAILED");
  }

  _mqttClient.unsubscribe(_stateTopic);
  _mqttClient.disconnect();
  unsigned long start = millis();
  while (millis() - start < 1000)
  {
    _mqttClient.loop();
    delay(10);
    yield();
  }

  _wifiClient.clear();
  _wifiClient.stop();
  delay(200);

  Serial.println(_stateTopic);
  Serial.println(statePayload);
}

void SendDataToThingSpeak()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("TS: No WiFi, skipping.");
    return;
  }

  Serial.printf("TS temp=%0.2f hum=%0.2f rssi=%d\n", _temperature, _humidity, _rssi);

  // Construct the full URL
  String url = "http://api.thingspeak.com/update?api_key=" + _tsKey;
  url += "&field1=" + String(_temperature, 2);
  url += "&field2=" + String(_humidity, 2);
  url += "&field3=" + String(_rssi);

  bool success = false;

  for (int attempt = 1; attempt <= sendRetries; attempt++)
  {
    HTTPClient http;
    Serial.printf("TS: Attempt %d of %d... ", attempt, sendRetries);

    // Reuse is false because we are likely going to sleep after this
    http.setReuse(false);
    http.useHTTP10(true);   // Use HTTP/1.0 to ensure the server closes the connection immediately after response
    http.setTimeout(15000); // 15 seconds gives the C3 enough time for ARP resolution
    http.addHeader("Connection", "close");

    if (http.begin(url))
    {
      int httpCode = http.GET();

      if (httpCode == HTTP_CODE_OK)
      { // HTTP 200
        String payload = http.getString();
        payload.trim();

        if (payload == "0")
        {
          Serial.println("FAILED (Rate Limited)");
        }
        else
        {
          Serial.printf("SUCCESS! Entry ID: %s\n", payload.c_str());
          success = true;
        }
      }
      else
      {
        Serial.printf("FAILED (HTTP Error: %d - %s)\n",
                      httpCode, http.errorToString(httpCode).c_str());
      }

      http.end(); // closes the socket and releases memory
    }
    else
    {
      Serial.println("FAILED (Unable to connect to host)");
    }

    if (success)
      break;

    // Purge delay if we fail
    if (attempt < sendRetries)
    {
      Serial.println("Waiting for purge...");

      NonBlockingDelay(2000);
    }
  }
}

bool WaitForNetwork()
{
  unsigned long start = millis();
  while (millis() - start < 5000)
  {
    if (WiFi.status() == WL_CONNECTED && WiFi.gatewayIP().toString() != "0.0.0.0")
    {
      Serial.println("WiFi connected and gateway IP valid");
      start = millis();
      while (millis() - start < 500)
      {
        delay(10);
      }
      return true;
    }
    else
    {
      delay(200);
    }
  }
  Serial.println("Network check failed: WiFi not connected or gateway IP invalid");
  return false;
}

bool ConnectToWiFi(void)
{
  WiFi.persistent(false); // Don't save credentials to flash (we manage this ourselves)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE);

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connecting using DHCP...");
    Serial.printf("ssid: %s, pass: %s\n", _ssid.c_str(), _password.c_str());

    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);

    delay(100); // Short delay before starting connection attempt
    WiFi.begin(_ssid.c_str(), _password.c_str());

    unsigned long startWait = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startWait < 7000))
    {
      yield();
      delay(10);
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    else
    {
      Serial.println("All connection attempts failed.");
      return false;
    }
  }
  else
  {
    Serial.print("Already connected!");
    return true;
  }
}

void Sleep(void)
{
  WiFi.disconnect(true, true); // Disconnect and erase credentials to ensure a clean slate for next connection
  unsigned long startWait = millis();
  while (WiFi.isConnected() && (millis() - startWait < 2000))
  {
    delay(10);
  }
  WiFi.mode(WIFI_OFF);
  delay(250);
  // Normalizing thermal footprint: ESP32-C3 self-heating is constant if uptime is fixed
  while (millis() - _startingMillis < 15000)
  {
    yield();
    delay(10);
  }
  Serial.flush();
  int32_t timeToSleep = _timerDelay - millis() / 1000;
  if (timeToSleep <= 0)
  {
    timeToSleep = 10; // Minimum sleep of 10 seconds to prevent immediate wake loops
  }
  Serial.printf("Entering deep sleep for %ld seconds...\n\n", timeToSleep);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(timeToSleep * 1000000ULL);
  esp_deep_sleep_start();
}

bool IsNumeric(String s, bool isPositive)
{
  if (s.length() == 0)
    return false;

  bool decimalPoint = false;

  for (int i = 0; i < s.length(); i++)
  {
    char c = s[i];

    // Handle minus
    if (c == '-')
    {
      if (isPositive)
        return false; // reject minus if only positives allowed
      if (i != 0)
        return false; // minus only at start
      if (s.length() == 1)
        return false; // string can't be just "-"
    }
    // Handle decimal point
    else if (c == '.')
    {
      if (decimalPoint)
        return false; // only one decimal point allowed
      decimalPoint = true;
    }
    // Must be digit
    else if (!isdigit(c))
    {
      return false;
    }
  }

  // Reject only decimal point strings or "-."
  if (s == "." || s == "-.")
    return false;

  return true;
}

bool IsPositiveInteger(String s)
{

  if (s.length() == 0)
    return false;

  for (int i = 0; i < s.length(); i++)
  {
    if (!isdigit(s[i]))
      return false;
  }

  return true;
}

void NonBlockingDelay(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    yield();
    delay(10);
  }
}