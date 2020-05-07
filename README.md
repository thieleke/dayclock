![Day Clock](https://github.com/thieleke/dayclock/blob/master/frontview.jpg)


# Dayclock
ESP32 based day clock + sensors

This is mixup of https://github.com/phreakmonkey/DayClock 's 3D printed pointer and hemispherical day clock with my code to move the pointer within the day, as well as integrating with a DHT temperature/humdity sensor and a MHZ-19B carbon dioxide sensor and providing HTTP endpoints (HTML/XML/JSON) and pushing data out via UDP.

Parts needed for this design:

* ESP32 (ESP8266 should work, but is untested).
* DHT11 or DHT22 (preferred) temperature/humidity sensor.  Lots of these online - AdaFruit sells the sensor and a resistor, but you might prefer an integrated package with the circuit board and resistor built in.  Search on eBay, etc.  Expect to spend no more than $10 and probably much less.
* MHZ-19B CO2 sensor.  These are all over eBay and the usual sources.  Look around and don't spend more than around $25 for a nicely packaged version.
* LCD panel with I2C interface.  These are typically 1602a type devices and should cost a few dollars or so on the usual places.  The code defaults to a 16x2 display - larger displays might work but YMMV.  There are plenty of 3D designs available to hold these LCD screens - I used and liked https://www.thingiverse.com/make:440729
* 3D printed versions of phreakmonkey's https://github.com/phreakmonkey/DayClock/blob/master/DayClock.stl and https://github.com/phreakmonkey/DayClock/blob/master/DayClock-hand.stl
* A generic "9G" servo motor or equivalent.


Configuration:
The Arduino .ido file has a configuration section at the top of the code.  You will need to configure, at least:
* `ssid` and `password` - set these to your WiFi network's settings
* `gmtOffset_sec` and `daylightOffset_sec` - these will depend on your location.  

as well as:

* You will probably also need to adjust the `dayPos` and `SERVO_MIN` and `SERVO_MAX` values, although this can wait until you have phreakmonkey's parts printed and assembled with the servo.  The default values work well with my servo/print, but you will likely need to adjust some or all of these values.
* You may also wish to uncomment the `DISPLAY_CELSIUS` to use celsius units for the LCD and HTTP interfaces.
* If you plan to have the UDP XML push messages enabled, set up `UDP_HOST_IP` and `UDP_HOST_PORT`.

The default pinout configuration is:
* Servo:  GPIO Pin 13 
* DHT: 5V, GND, GPIO Pin 14
* LCD: 5V, GND, SDA -> I2C SDA, SCL -> IC2 SCL
* MH-Z19B: RX -> TX2, TX -> RX2

Software / Libraries used (and much thanks given!):
* https://github.com/adafruit/Adafruit_Sensor
* https://github.com/espressif/arduino-esp32
* https://github.com/adafruit/DHT-sensor-library
* https://github.com/jkb-git/ESP32Servo
* https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
* https://github.com/WifWaf/MH-Z19
* https://github.com/me-no-dev/ESPAsyncWebServer/

Suggestions:
* If you're using a small breadboard and a ESP32 with male pins, remove the +/- rail on the breadboard and snap two breadboards together so the ESP32 module will fit with room to wire up the connections.
* MHZ-19B is pretty touchy - read https://github.com/WifWaf/MH-Z19/blob/master/README.md for the details. 
* Superglue phreakmonkey's dayclock print to the niq_ro's 1602a stand print to make a nice integrated unit.  Then stick down the DHT and MHZ-19B to the breadboard.  There are some nice 3D prints on Thingiverse for the DHT models - I like https://www.thingiverse.com/make:679955 which, scaled up, also works well for the MHZ-19B sensor.

HTTP - there are 3 views into the DayClock's data
* HTTP: a webserver is running on the ESP that defaults to showing the current temperature/humidity/CO2 level/timestamp.  This will either reload every 5 seconds for a non-Javascript browser or will update automatically.
* /xml - this is a XML representation of the last sensor read values
* /json - this is the same data as XML, provided in JSON format.  This is also used by the default handler to automatically update the page.


