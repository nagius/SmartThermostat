# SmartThermostat 

SmartThermostat is an Arduino sketch for the ESP8266 module that provide automatic control of one eletric heater over an HTTP REST-like API.
The goal is to turn a dumb mechanical heater into a WiFi heater controllable by any home automation system, like [Heatman](https://github.com/nagius/heatman).

SmartThermostat provide two modes : 

 - Hysteresis mode :

This is the traditional thermostat. You set the desired temperature, when the room temperature is below, it switch on, when it's above, it switch off. 

 - PID controller mode :

This use the so called proportional–integral–derivative algorithm to adjust the temperature with more precision and smoother transitions. This mode modulate the power of the heater by simulating PWM at ultra-low frequency (200 mHz).
The tuning of this algorithm can be tricky. Refer to http://newton.ex.ac.uk/teaching/CDHW/Feedback/ for more information on temperature control algorithm.

NOTE: A Solid State Relay is recommended for the PID mode, as the on/off cycle will be frequent.


## Hardware 

This sketch is designed to be use with the following hardware :

 - One ESP8266-01
 - One or more thermal sensors DS18B20
 - One relay or SSR rated for AC load

Make sure this relay can handle the current drawn by your heater. For a 1000W heater under 230V AC, that's almost 5A. I suggest at least a 10A relay or a 20A SSR to be sure (cheap chinese components are often over-rated).

Be careful when plugin the ESP8266 into a serial port, this chip work at 3.3V and most adapter use 5V (like Arduino or FTDI serial adapter).
If you don't have a USB serial adapter with level shifter, the Raspberry Pi's UART will do fine as it work on 3.3V.

### More info

 - KiCad schematics for the detailled wiring : [Flashing mode](docs/flashing-mode.jpg), [Execution mode](docs/execution-mode.jpg).
 - ESP8266 wiring and bootstrap: http://www.esp8266.com/wiki/doku.php?id=getting-started-with-the-esp8266
 - USB +5v filtering: http://andybrown.me.uk/2015/07/24/usb-filtering/

## Tools installation

The compilation of this sketch require the Arduino IDE with the ESP8266 libraries.

Refer to the [Arduino core for ESP8266](https://github.com/esp8266/Arduino) for the installation steps.

You may also need the esptool CLI, available on [Espressif website](https://github.com/espressif/esptool).


## Firmware compilation and upload

In the Arduino IDE, open the SmartThermostat sketch.
Edit the main file and set the two defines `DEFAULT_SSID` and `DEFAULT_KEY` according to your WiFi setup.

To install dependancies, go to 'Sketch' -> 'Include Library' -> 'Manage Library' and install these packages :

 - OneWire
 - DallasTemperature
 - PID

In the 'Tools' menu, select the board 'Generic ESP8266 board' and set these settings :

 - Flash mode : QIO
 - Flash frequency: 40 Mhz
 - CPU Frequency: 80 Mhz
 - Flash size :  1M (64K SPIFFS)

Then run 'Compile' and 'Upload'.

If the ESP8266 module is not plugged into the same computer, you can also use the following command to upload the compiled binary :

```
esptool.py --port /dev/ttyAMA0 write_flash -fm qio 0x00000 SmartThermostat.ino.generic.bin
```


## Debug and monitor serial output

The serial communication must be set to `115200 8N1`. Any serial console software will do the job :

```
minicom -D /dev/ttyAMA0
```

Using the [execution mode](docs/execution-mode.jpg) wiring, plug the serial port and power the module, you should see a boot message like this on the console :

```
SmartThermostat v1.0 started.
[0.271] Loaded settings from flash
[0.452] Found 3 DS18B20 devices.
[0.467] Using device at index 0 [28-0416549140ff].
[0.467] HTTP server started.
[0.468] Connecting to WiFi MyAP
...........OK
[5.969] IP address: 192.168.1.3
```

This log is also available from your browser at this URL :
  
```
http://<esp8266_IP>/debug
```

You can enable debug mode to get more information :

```
curl -X POST -F 'debug=true' http://<esp8266_IP>/settings
```

## Authentication

SmartThermostat support HTTP Auth Basic authentication (but not https).

By default, there is no login required. Everybody on your network will be able to control your heater.
To enable authentication, you have to set both settings `login` and `password` :

```
curl -X POST 'http://<esp8266_IP>/settings?login=admin&password=mysecret'
```

All settings are stored into flash and are persistent upon reboot and power loss.

### How to reset credentials (and everything else)

If you loose the password, there is no way to recover it. Even flashing again the firmware will keep the previously saved settings.
The only way is to completely wipe out the flash. 

```
esptool.py --port /dev/ttyAMA0 erase_flash
```

You will need to re-upload the firmware after this operation.

## API definition

 - GET /state

Show the current mode, requested target temperature and actual temperature of the room. Temperatures are given in °C with 2 digits precision.

  * Return "application/json" :

```json
{
  "mode": "off",
  "target": 19.30,
  "actual": 20.56
}
```

 - GET /settings

Show the current settings, as stored in flash. See `POST /settings` for details on each parameter.

  * Return "application/json" :

```json
{
  "hysteresis": 0.1,
  "offset": 0,
  "login": "",
  "debug": false,
  "index": 0,
  "Kp": 6,
  "Ki": 3,
  "Kd": 2,
  "ssid": "MyAP"
}
```

 - GET /debug

Display device information and last 100 lines of the log. Can be quite verbose if debug mode is on.

   * Return "text/plain" :

```
 ==== DEBUG LOG ====
Chip ID: 14043137
Free Heap: 20880
Flash Size: 1048588
Uptime: 00:50:27
Printing last 100 lines of the log:
[0.271] Loaded settings from flash
[0.452] Found 3 DS18B20 devices.
[0.467] Using device at index 0 [28-0416549140ff].
[0.467] HTTP server started.
[0.468] Connecting to WiFi MyAP
[5.969] IP address: 192.168.1.3
[989.620] Connecting to WiFi MyAP
[993.620] IP address: 192.168.1.3
 ==== END LOG ====
```

 - POST /state

Update requested state. This is volatile and won't be kept after a reboot. At boot time, default value are mode = off, target = 0

   * Parameters :

     - mode : *[on|off|pid|hysteresis]*
     - target : *[float]*

The mode 'on' switch on the heater regardless of the temperature. The heater may overheat if it doesn't have a thermal safety or a built-in thermostat.
Target is the requested temperature, in degree Celcius.

  * Return :

Return the same information as  `GET /state`.

   * Example :

```
curl -X POST -F 'target=19.3' -F 'mode=pid' http://<esp8266_IP>/state
```

 - POST /settings

Update configuration settings. This is stored in flash and is kept after a power loss.

   * Parameters :

     - hysteresis : *[float]*	Hysteresis tunning, precision in °C. (default 0.1)
     - offset : *[float]*	Offset (in °C) added to the sensor's value for manual calibration. (default 0)
     - debug : *[bool]*		Turn on extra logging. (default false)
     - index : *[int]*		Sensor's index on the OneWire bus. Used to select active sensor if multiple available. (default 0)
     - login : *[str]*		Auth Basic login. (default empty)
     - password : *[str]*	Auth Basic password. (default empty)
     - Kp : *[float]*		PID tunning, Proportionnal component.
     - Ki : *[float]*		PID tunning, Integral component.
     - Kd : *[float]*		PID tunning, Derivative component.
     - ssid : *[str]*		SSID of the AP to connect to.
     - key : *[str]*		WPA2-PSK of the AP to connect to.
     - active : *<high|low>*	GPIO output to the heater. Set according to your wiring. (default low)

Both 'ssid' and 'key' need a reboot to be applied and connect to the new AP.

   * Return :

Return the same information as `GET /settings`.

   * Examples :

```
curl -X POST -F 'login=admin' -F 'password=mysecret' http://<esp8266_IP>/settings
curl -X POST -F 'offset=0.315' -u admin:mysecret http://<esp8266_IP>/settings
```

## License 

Copyleft 2017 - Nicolas AGIUS - GNU GPLv3
