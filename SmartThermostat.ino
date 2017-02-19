/*************************************************************************
 *
 * This file is part of the SmartThermostat Arduino sketch.
 * Copyleft 2017 Nicolas Agius <nicolas.agius@lps-it.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ***********************************************************************/

/*
 * For details regarding PID's settings Kp Ki Kd, See :
 *  - http://playground.arduino.cc/Code/PIDLibrary
 *  - http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 *  - http://newton.ex.ac.uk/teaching/CDHW/Feedback/
 */

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include "Logger.h"

// Configuration
#define GPIO_ONEWIRE 0          // DS18B20 pin
#define GPIO_RELAY 2            // GPIO Ouput pin for relay (active low)

// Default value
#define DEFAULT_HYSTERESIS 0.1  // 0.2 °C wide
#define DEFAULT_PID_KP 6        // PID settings
#define DEFAULT_PID_KI 3
#define DEFAULT_PID_KD 2
#define DEFAULT_INDEX 0         // Index of the DS18B20 sensor on the One-Wire bus
#define DEFAULT_OFFSET 0        // Offset in °C for manual sensors calibration
#define DEFAULT_LOGIN ""        // AuthBasic credentials
#define DEFAULT_PASSWORD ""     // (default no auth)
#define DEFAULT_SSID "XXXX"     // Default Wifi SSID
#define DEFAULT_KEY "XXXX"      // Default Wifi WPA2-PSK


// Internal constant
#define ONEWIRE_ADDR_LEN 16     // 6 bytes + 3 chars header + EOS = 16 chars
#define AUTHBASIC_LEN 21        // Login or password 20 char max
#define SSID_LEN 32             // Max SSID length as 802.11 definition
#define KEY_LEN 63              // Max WPA2-PSK length
#define PWM_PERIOD 5000         // Period for slow PWM : 5s ( That's 200 mHz ) 
#define FREQ_PID 250            // Run PID every 250 ms
#define FREQ_HYSTERESIS 5000    // Run hysteresis every 5s

// Mode definition
#define MODE_OFF 1
#define MODE_HYSTERESIS 2
#define MODE_PID 3
#define MODE_ON 4

struct ST_STATE {
  uint8_t mode;
  double target;
  unsigned long lastRun;
  bool heat;
};

struct ST_STATE_FLAGS {
  bool mode;
  bool target;
};

struct ST_SETTINGS {
  double hysteresis;
  double offset;
  uint8_t index;
  bool debug;
  char login[AUTHBASIC_LEN];
  char password[AUTHBASIC_LEN];
  double Kp;
  double Ki;
  double Kd;
  char ssid[SSID_LEN];
  char key[KEY_LEN];
};

struct ST_SETTINGS_FLAGS {
  bool hysteresis;
  bool offset;
  bool index;
  bool debug;
  bool login;
  bool password;
  bool pid;
  bool ssid;
  bool key;
};

struct ST_PID {
  double input;
  double output;
  unsigned long windowStartTime;
};

// Global variables
OneWire oneWire(GPIO_ONEWIRE);
DallasTemperature sensors(&oneWire);
ESP8266WebServer server(80);
Logger logger = Logger();

ST_SETTINGS settings;
ST_STATE state = { MODE_OFF, 0.0, 0, false }; // Startup mode is OFF
ST_PID pid;

// Create PID driver
PID myPID(&(pid.input), &(pid.output), &(state.target), DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD, DIRECT);



/**
 * HTTP route handlers
 ********************************************************************************/

/**
 * GET /
 */
void handleGETRoot() 
{
  // I always loved this HTTP code
  server.send(418, "text/plain", "\
            _           \r\n\
         _,(_)._            \r\n\
    ___,(_______).          \r\n\
  ,'__.           \\    /\\_  \r\n\
 /,' /             \\  /  /  \r\n\
| | |              |,'  /   \r\n\
 \\`.|                  /    \r\n\
  `. :           :    /     \r\n\
    `.            :.,'      \r\n\
      `-.________,-'        \r\n\
  \r\n");
}

/**
 * GET /debug
 */
void handleGETDebug()
{
  String msg = logger.getLog();
  
  Serial.print(msg);
  server.send(200, "text/plain", msg);
}

/**
 * GET /settings
 */
void handleGETSettings()
{
  if(!isAuthBasicOK())
    return;
 
  server.send(200, "application/json", getJSONSettings());
}

/**
 * POST /settings
 * Args :
 *   - hysteresis = <float>
 *   - offset = <float>
 *   - debug = <bool>
 *   - index = <int>
 *   - login = <str>
 *   - password = <str>
 *   - Kp = <float>
 *   - Ki = <float>
 *   - Kd = <float>
 *   - ssid = <str>
 *   - key = <str>
 */
void handlePOSTSettings()
{
  ST_SETTINGS st;
  ST_SETTINGS_FLAGS isNew = { false, false, false, false, false, false, false, false, false };

  if(!isAuthBasicOK())
    return;

  // Init PID settings with current ones
  st.Kp = settings.Kp;
  st.Ki = settings.Ki;
  st.Kd = settings.Kd;

  // Check if args have been supplied
  if(server.args() == 0)
  {
    server.send(400, "test/plain", "Invalid parameters\r\n");
    return;
  }

  // Parse args   
  for(uint8_t i=0; i<server.args(); i++ ) 
  {
    String param = server.argName(i);
    if(param == "hysteresis")
    {
      st.hysteresis = server.arg(i).toFloat();
      if(st.hysteresis <= 0)
      {
        server.send(400, "text/plain", "Bad value, hysteresis must be a positive non-zero float.\r\n");
        return;
      }
      isNew.hysteresis = true;
    }
    else if(param == "offset")
    {
      st.offset = server.arg(i).toFloat();
      isNew.offset = true;
    }
    else if(param == "debug")
    {
      st.debug = server.arg(i).equalsIgnoreCase("true");
      isNew.debug = true;
    }
    else if(param == "index")
    {
      st.index = server.arg(i).toInt();
      if(st.index < 0)
      {
        server.send(400, "text/plain", "Bad value, index must be a positive int.\r\n");
        return;
      }
      isNew.index = true;
    }
    else if(param == "login")
    {
      server.arg(i).toCharArray(st.login, AUTHBASIC_LEN);
      isNew.login = true;
    }
    else if(param == "password")
    {
      server.arg(i).toCharArray(st.password, AUTHBASIC_LEN);
      isNew.password = true;
    }
    else if(param == "Kp")
    {
      st.Kp = server.arg(i).toFloat();
      isNew.pid = true;
    }
    else if(param == "Ki")
    {
      st.Ki = server.arg(i).toFloat();
      isNew.pid = true;
    }
    else if(param == "Kd")
    {
      st.Kd = server.arg(i).toFloat();
      isNew.pid = true;
    }
    else if(param == "ssid")
    {
      server.arg(i).toCharArray(st.ssid, SSID_LEN);
      isNew.ssid=true;
    }
    else if(param == "key")
    {
      server.arg(i).toCharArray(st.key, KEY_LEN);
      isNew.key=true;
    }
    else
    {
      server.send(400, "text/plain", "Unknown parameter: " + param + "\r\n");
      return;
    }
  }

  // Save changes
  if(isNew.hysteresis)
  {
    settings.hysteresis = st.hysteresis;
    logger.info("Updated hysteresis to \"%s\".", String(st.hysteresis, 3).c_str());
  }

  if(isNew.index)
  {
    settings.index = st.index;
    logger.info("Updated index to %d.", st.index);
  }

  if(isNew.offset)
  {
    settings.offset = st.offset;
    logger.info("Updated offset to %s.", String(st.offset, 3).c_str());
  }

  if(isNew.debug)
  {
    settings.debug = st.debug;
    logger.setDebug(st.debug);
    logger.info("Updated debug to %s.", st.debug ? "true" : "false");
  }

  if(isNew.login)
  {
    strcpy(settings.login, st.login);
    logger.info("Updated login to \"%s\".", st.login);
  }

  if(isNew.password)
  {
    strcpy(settings.password, st.password);
    logger.info("Updated password.");
  }

  if(isNew.pid)
  {
    settings.Kp = st.Kp;
    settings.Ki = st.Ki;
    settings.Kd = st.Kd;
    myPID.SetTunings(st.Kp, st.Ki, st.Kd);
    logger.info("Updated PID settings Kp:%s Ki:%s Kd:%s", String(st.Kp, 3).c_str(), String(st.Ki, 3).c_str(), String(st.Kd, 3).c_str());
  }

  if(isNew.ssid)
  {
    strcpy(settings.ssid, st.ssid);
    logger.info("Updated SSID to %s", st.ssid);
  }

  if(isNew.key)
  {
    strcpy(settings.key, st.key);
    logger.info("Updated Key");
  }

  saveSettings(); 
  
  // Reply with current settings
  server.send(201, "application/json", getJSONSettings());
}

/**
 * GET /state
 */
void handleGETState()
{
  if(!isAuthBasicOK())
    return;

  server.send(200, "application/json", getJSONState());
}

/**
 * POST /state
 * Args :
 *   - mode = [on|off|hysteresis|pid]
 *   - target = <float>
 */
void handlePOSTState()
{
  ST_STATE st;
  ST_STATE_FLAGS isNew = { false, false };
  
  if(!isAuthBasicOK())
    return;

  // Check if args have been supplied
  if(server.args() == 0)
  {
    server.send(400, "test/plain", "Invalid parameters\r\n");
    return;
  }
  
  // Parse args   
  for(uint8_t i=0; i<server.args(); i++ ) 
  {
    String param = server.argName(i);
    if(param == "mode")
    {
      String strMode = server.arg(i);
      isNew.mode = true;
      
      if(strMode == "on")
      {
        st.mode = MODE_ON;
      }
      else if(strMode == "off")
      {
        st.mode = MODE_OFF;
      }
      else if(strMode == "hysteresis")
      {
        st.mode = MODE_HYSTERESIS;
      }
      else if(strMode == "pid")
      {
        st.mode = MODE_PID;
      }
      else
      {
        isNew.mode = false;
        server.send(400, "text/plain", "Invalid mode: " + strMode + "\r\n");
        return;
      }
    }
    else if(param == "target")
    {
      st.target = server.arg(i).toFloat();
      if(st.target != 0)  // Bad parsing return 0
      {
        isNew.target = true;
      }
      else
      {
        server.send(400, "text/plain", "Bad value, target must be a non-zero float.\r\n");
        return;
      }
    }
    else
    {
      server.send(400, "text/plain", "Unknown parameter: " + param + "\r\n");
      return;
    }
  }

  // Apply change when all args have been parsed successfully
  if(isNew.target)
    setTarget(st.target);

  if(isNew.mode)
    setMode(st.mode);

  // Reply with the new state
  server.send(201, "application/json", getJSONState());
}

/**
 * WEB helpers 
 ********************************************************************************/

bool isAuthBasicOK()
{
  // Disable auth if not credential provided
  if(strlen(settings.login) > 0 && strlen(settings.password) > 0)
  {
    if(!server.authenticate(settings.login, settings.password))
    {
      server.requestAuthentication();
      return false;
    }
  }
  return true;
}

String getJSONSettings()
{
  //Generate JSON 
  String json = "{ \"hysteresis\": ";
  json += settings.hysteresis;
  json += ", \"offset\": ";
  json += settings.offset;
  json += ", \"login\": \"";
  json += settings.login;
  json += "\", \"debug\": ";
  json += settings.debug ? "true" : "false";
  json += ", \"index\": ";
  json += settings.index;
  json += ", \"Kp\": ";
  json += settings.Kp;
  json += ", \"Ki\": ";
  json += settings.Ki;
  json += ", \"Kd\": ";
  json += settings.Kd;
  json += ", \"ssid\": \"";
  json += settings.ssid;
  json += "\" }\r\n";

  return json;
}

String getJSONState()
{  
  // Generate JSON blob
  String json = "{ \"mode\": \"";
  json += getMode();
  json += "\", \"target\": ";
  json += state.target;
  json += ", \"actual\": ";
  json += getTemp();
  if(state.mode == MODE_PID)
  {
    json += ", \"power\": ";
    json += pid.output;   // Display output power in %
  }
  json += " }\r\n";

  return json;
}


/**
 * General helpers 
 ********************************************************************************/

void connectWiFi()
{
  WiFi.begin(settings.ssid, settings.key);
  logger.info("Connecting to WiFi %s", settings.ssid);

  // Wait for connection
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("OK");

  // Display local ip
  logger.info("IP address: %s", WiFi.localIP().toString().c_str());
}

void setTarget(double target)
{
  state.target = target;
  logger.info("Target tempetature set to %s", String(target, 3).c_str());
}

void setMode(uint8_t mode)
{
  if(state.mode != mode)
  {
    state.mode=mode;
    switch(mode)
    {
      case MODE_ON:
        switchOn();
        break;
  
      case MODE_PID:
        pid.windowStartTime = millis();
        break;
        
      case MODE_HYSTERESIS:
        // Nothing to do
        break;
  
      case MODE_OFF:
      default:        // Default to OFF in case of internal error
        switchOff();
        break;
    }

    logger.info("Mode set to %s", getMode().c_str());    
  }
}

String getMode(void)
{
  String strMode;
  
  // Convert mode to String
  switch(state.mode)
  {
    case MODE_ON:
      strMode="on";
      break;
      
    case MODE_HYSTERESIS:
      strMode="hysteresis";
      break;
      
    case MODE_PID:
      strMode="pid";
      break;

    // Undefined mode is considered OFF
    default:
      strMode="off";
  }

  return strMode;
}

void switchOn()
{
  if(!state.heat)
  {
    state.heat = true;
    digitalWrite(GPIO_RELAY, LOW);
    logger.info("Heater ON");
  }
}

void switchOff()
{
  if(state.heat)
  {
    state.heat = false;
    digitalWrite(GPIO_RELAY, HIGH);
    logger.info("Heater OFF");
  }
}


/**
 *  One-wire helpers 
 ********************************************************************************/

 
char *getDeviceAddress(char *str,  uint8_t index)
{
  DeviceAddress addr;
  if(sensors.getAddress(addr, index))
  {
    snprintf(str, ONEWIRE_ADDR_LEN, "%02x-%02x%02x%02x%02x%02x%02x", addr[0], addr[6], addr[5],addr[4], addr[3], addr[2], addr[1]);
  }
  else
  {
    logger.debug("Onewire address not found at index %i", index);
    str[0]='\0';
  }
  return str;
}


double getTemp()
{
  double temp;
  double offset = 0;
  char addr[ONEWIRE_ADDR_LEN];
  uint8_t index = settings.index;
  int max_try=10;
  
  do
  {
    sensors.requestTemperaturesByIndex(index); 
    delay(10); 
    temp = sensors.getTempCByIndex(index);
    logger.debug("Raw temperature: %s", String(temp, 3).c_str());
    max_try--;
  }
  while ((temp == 85.0 || temp == (-127.0)) && max_try > 0);

  if (max_try <= 0)
  {
    logger.debug("Error reading from DS18B20 at index %d", index);
  }
  else
  {
    // Perform one-point calibration with offset
    if(settings.offset != 0)
    {
      // Manual calibration
      offset = settings.offset;
    }
    else
    {
      // Builtin calibration for known sensors
      getDeviceAddress(addr, index);
      if(strlen(addr)>0)
      {
        if(strcmp(addr, "28-0416549140ff") == 0)
        {
          offset = -0.311;
        }
        else if(strcmp(addr, "28-0316442b74ff") == 0)
        {
          offset = -0.398;
        }
        else if(strcmp(addr, "28-04168438ddff") == 0)
        {
          offset = 0.001;
        }
        else
        {
          logger.debug("No calibration data available for %s.", addr);
        }
      }
      else
      {
        logger.debug("Can't get sensor address for calibration.");
      }
    }

    // Apply selected calibration offset
    temp += offset;
    logger.debug("Applying offset %s. Corrected temp is %s.",String(offset, 3).c_str(), String(temp, 3).c_str());
  }

  return temp;
}

/**
 * Flash memory helpers 
 ********************************************************************************/

void saveSettings()
{
  uint8_t buffer[sizeof(settings) + 1];  // Use the last byte for CRC

  memcpy(buffer, &settings, sizeof(settings));
  buffer[sizeof(settings)] = OneWire::crc8(buffer, sizeof(settings));

  for(int i=0; i < sizeof(buffer); i++)
  {
    EEPROM.write(i, buffer[i]);
  }
  EEPROM.commit();
}

void loadSettings()
{
  uint8_t buffer[sizeof(settings) + 1];  // Use the last byte for CRC

  for(int i=0; i < sizeof(buffer); i++)
  {
    buffer[i] = uint8_t(EEPROM.read(i));
  }

  // Check CRC
  if(OneWire::crc8(buffer, sizeof(settings)) == buffer[sizeof(settings)])
  {
    memcpy(&settings, buffer, sizeof(settings));
    logger.setDebug(settings.debug);
    logger.info("Loaded settings from flash");

    // Display loaded setting on debug
    logger.debug("FLASH: %s", getJSONSettings().c_str());
  }
  else
  {
    logger.info("Bad CRC, loading default settings.");
    settings.hysteresis = DEFAULT_HYSTERESIS;
    settings.index = DEFAULT_INDEX;
    settings.offset = DEFAULT_OFFSET;
    strcpy(settings.login, DEFAULT_LOGIN);
    strcpy(settings.password, DEFAULT_PASSWORD);
    settings.debug = false;
    settings.Kp = DEFAULT_PID_KP;
    settings.Ki = DEFAULT_PID_KI;
    settings.Kd = DEFAULT_PID_KD;
    strcpy(settings.ssid, DEFAULT_SSID);
    strcpy(settings.key, DEFAULT_KEY);
    saveSettings();
  }
}


/**
 * Thermal control
 ********************************************************************************/

void runHysteresis()
{
  double temp = getTemp();

  logger.debug("Hysteresis: target=%s current=%s", String(state.target, 3).c_str(), String(temp, 3).c_str());
  if(temp == 85.0 || temp == (-127.0))
  {
    logger.debug("Sensor not reliable. Nothing to do");
  }
  else
  {
    if(temp > state.target + settings.hysteresis)
    {
      switchOff();
    }
  
    if(temp < state.target - settings.hysteresis)
    {
      switchOn();
    }
  }
}

void runPID()
{
  double temp = getTemp();
  unsigned long now = millis();
  
  if(temp != -127.0)
  {
    pid.input = temp;
    myPID.Compute();
    logger.debug("PID: target=%s current=%s output=%s", String(state.target,3).c_str(), String(pid.input, 3).c_str(), String(pid.output, 3).c_str());
  }
  
  // Shift the PWM window
  if(now - pid.windowStartTime > PWM_PERIOD)
  {
    pid.windowStartTime += PWM_PERIOD;
  }
  
  // Simulate low frequency PWM
  if((pid.output * PWM_PERIOD)/100 > now - pid.windowStartTime)
  {
    switchOn();
  }
  else
  {
    switchOff();
  }
}



/**
 * Main
 ********************************************************************************/

void setup(void)
{
  // INIT
  pinMode(GPIO_RELAY, OUTPUT);
  digitalWrite(GPIO_RELAY, HIGH);
  Serial.begin(115200);
  EEPROM.begin(512);
  Serial.println("\r\nSmartThermostat v1.0 started.");
  WiFi.mode(WIFI_STA);

  // Load settigns from flash
  loadSettings();
  
  // Init DS18B20
  sensors.begin();
  logger.info("Found %d DS18B20 devices.", sensors.getDeviceCount());

  // Get selected One-wire device address
  char addr[ONEWIRE_ADDR_LEN]; // 8 bytes address
  getDeviceAddress(addr, settings.index);
  if(strlen(addr)>0)
    logger.info("Using device at index %d [%s].", settings.index, addr);

  // Setup HTTP handlers
  server.on("/", handleGETRoot );
  server.on("/debug", HTTP_GET, handleGETDebug);
  server.on("/state", HTTP_GET, handleGETState);
  server.on("/state", HTTP_POST, handlePOSTState);
  server.on("/settings", HTTP_GET, handleGETSettings);
  server.on("/settings", HTTP_POST, handlePOSTSettings);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
  logger.info("HTTP server started.");

  // Init PID driver
  myPID.SetOutputLimits(0, 100);  // Give output as power percentage
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(FREQ_PID);
  myPID.SetTunings(settings.Kp, settings.Ki, settings.Kd);
}


void loop(void)
{
  // Reconnect automatically
  if(WiFi.status() != WL_CONNECTED)
    connectWiFi();
  
  server.handleClient();

  // Run manual timer here as Ticker is not reliable
  unsigned long now = millis();
  unsigned long elapsed = (now - state.lastRun);

  switch(state.mode)
  {
    case MODE_PID:  
      if(elapsed >= FREQ_PID)
      {
        runPID();
        state.lastRun = now;
      }
      break;

    case MODE_HYSTERESIS:
      if(elapsed >= FREQ_HYSTERESIS)
      {
        runHysteresis();
        state.lastRun = now;
      }
      break;
  }    
}



