/*
   Default Pinouts
 *    * Servo:  GPIO Pin 13
 *    * DHT: 5V, GND, GPIO Pin 14
 *    * LCD: 5V, GND, SDA -> I2C SDA, SCL -> IC2 SCL
 *    * MH-Z19B: RX -> TX2, TX -> RX2
*/

/* Constants that you'll definitely want to update */
//#define DISPLAY_CELSIUS                         // Uncomment if you want the LCD and HTTP displays to default to Celsius temperature instead of Fahrenheit
#define UDP_HOST_IP            192,168,0,1        // IP address, comma separated (eg: 192,168,0,1), if you want to receive UDP push messages.
#define UDP_HOST_PORT          1234               // UDP port
const char *ssid               = "YOUR_SSID";     // Your WiFi SSID
const char *password           = "YOUR_PASSWORD"; // Your WiFi password
const char* ntpServer          = "pool.ntp.org";  // There are also region-specific addresses such as "us.pool.ntp.org"
const long  gmtOffset_sec      = -21600;          // Non-daylight saving time offset from UTC, in seconds (-18000 = US Eastern Standard Time, -21600 = US Central Standard Time, etc.)
const int   daylightOffset_sec = 3600;            // Number of seconds offset for daylight saving time (0 if daylight savings time if not applicable)

#define SERVO_PIN  13
#define DHT_PIN    14

// Uncomment the type of temperature sensor in use
//#define DHT_TYPE    DHT11     // DHT 11
#define DHT_TYPE      DHT22     // DHT 22 (AM2302)
//#define DHT_TYPE    DHT21     // DHT 21 (AM2301)

// Servo degree positons - fine tune to match your servo/display
//                     Sun  Mon  Tue  Wed  Thu  Fri  Sat
const int dayPos[8] = {180, 159, 133, 107, 80,  51,  26,  0};

// The Servo min/max values are basically magic numbers - experiment if the servo doesn't move as expected
#define SERVO_MIN 620
#define SERVO_MAX 3250

#define MHZ19_SERIAL_PORT 2   // RX2/TX2

#define LCD_I2C_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
/* End of constants that you'll definitely want to update */


#include <Arduino.h>
#include <Adafruit_Sensor.h>   // https://github.com/adafruit/Adafruit_Sensor/archive/master.zip
#include <AsyncTCP.h>          // https://github.com/me-no-dev/AsyncTCP/archive/master.zip
#include <AsyncUDP.h>          // https://github.com/espressif/arduino-esp32/archive/master.zip
#include <DHT.h>               // https://github.com/adafruit/DHT-sensor-library/archive/master.zip
#include <DHT_U.h>
#include <ESPAsyncWebServer.h> // https://github.com/me-no-dev/ESPAsyncWebServer/archive/master.zip
#include <ESP32Servo.h>        // https://github.com/jkb-git/ESP32Servo/archive/master.zip
#include <LiquidCrystal_I2C.h> // https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library/archive/master.zip
#include <MHZ19.h>             // https://github.com/WifWaf/MH-Z19/archive/master.zip
#include <time.h>
#include <Update.h>
#include <WiFi.h>
#include <Wire.h>

unsigned long ntpUpdated = 0;
char localTimeBuffer[30];
AsyncWebServer webServer(80);
AsyncUDP udp;
Servo servo;
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);  // Defaults to I2C address 0x27 and 16x2 display
DHT_Unified dht(DHT_PIN, DHT_TYPE);
#define NO_DATA_LOW -99999
#define NO_DATA_HIGH 99999
MHZ19 mhz19;
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
HardwareSerial mySerial(MHZ19_SERIAL_PORT);                // Create device to MH-Z19 serial (Default: RX2/TX2)
#define CO2_MIN  350
#define CO2_MAX  4000

unsigned int delayMS;
unsigned long sensorDelayTicks = 0;
unsigned long loops = 0;
unsigned long monitoringEndLoops;
bool showTempMinMax = true;
float minTemp = NO_DATA_HIGH;
float maxTemp = NO_DATA_LOW;
float minHumidity = NO_DATA_HIGH;
float maxHumidity = NO_DATA_LOW;
int   minCO2 = CO2_MAX;
int   maxCO2 = CO2_MIN;
int   co2Failures = 0;
float lastTemperature = 0;
float lastHumidity = 0;
int   lastCO2 = 0;
int   lastSensorTimestamp = 0;
char  lastSensorLocaltime[30];
size_t updateContentLen = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booted");

  // Initialize the LCD
  lcd.init();
  lcd.backlight();

  // Servo initialization
  Serial.println("Attaching to servo");
  servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);

  // Initialize DHT device
  dht.begin();

  // Connect to WiFi
  lcd.setCursor(0, 0);
  lcd.print("Connecting to        ");
  lcd.setCursor(0, 1);
  lcd.print(String(ssid) + "                    ");
  Serial.println("Connecting to Wi-Fi");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  lcd.setCursor(0, 0);
  lcd.print("Wi-Fi connected       ");
  lcd.setCursor(0, 1);
  lcd.print("                      ");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP().toString() + "               ");
  delay(2000);

  updateNTP();

#ifdef DEMO
  for (int i = 0; i < 8; i++)
  {
    lcd.setCursor(0, 0);
    lcd.print(String(i) + " -> " + String(dayPos[i]) + "               ");
    servo.write(dayPos[i]);
    delay(1500);
  }
#endif

  for (int i = 0; i < 8; i++)
  {
    Serial.println(String(i) + " -> " + String(dayPos[i]));
  }

  servo.write(dayPos[0]);

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));

  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));

  // Initialize MHZ-19B device
  mySerial.begin(BAUDRATE);
  mhz19.begin(mySerial);
  mhz19.autoCalibration();

  // Set delay between sensor readings based on sensor details
  delayMS = 5000; //(sensor.min_delay / 1000) * 2;
  monitoringEndLoops = (24 * 60 * 60) / (delayMS / 1000);

#ifdef UDP_HOST_IP
  // nc -u -l 1234
  if (udp.connect(IPAddress(UDP_HOST_IP), UDP_HOST_PORT)) {
    Serial.println("UDP connected");
    udp.onPacket([](AsyncUDPPacket packet) {
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length());
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      //reply to the client

      packet.printf("Got %u bytes of data\n", packet.length());

    });
    //Send unicast
    udp.print("Hello Server at " + String(localTime()) + " [" + String(millis()) + "] from " + WiFi.localIP().toString() + "\n");
    Serial.println(WiFi.localIP().toString());
  }
#else
  udp = NULL;
#endif

  webServer.on("/",            HTTP_GET,  httpRootHandler);
  webServer.on("/xml",         HTTP_GET,  httpXMLHandler);
  webServer.on("/json",        HTTP_GET,  httpJSONHandler);
  webServer.on("/favicon.ico", HTTP_GET,  httpFaviconHandler);
  webServer.on("/update",      HTTP_GET,  httpUpdateHandler);
  webServer.on("/do_update",   HTTP_POST, [](AsyncWebServerRequest * request) {},
  [](AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t *data, size_t len, bool final) {
    httpDoUpdateHandler(request, filename, index, data, len, final);
  }
              );
  webServer.begin();

#ifdef ESP32
  Update.onProgress(update_print_progress);
#endif

  lcd.setCursor(0, 0);
  lcd.print("Starting...             ");
}

void loop()
{
  if (updateContentLen > 0)
    return;

  unsigned long ticks = millis();

  if ((unsigned long)(ticks - sensorDelayTicks) >= delayMS)
  {
    sensorDelayTicks = ticks;

    // Update the LCD with temperature, humidity, and CO2
    print_sensors();

    struct tm t;
    getLocalTime(&t);
    move_pointer(t.tm_wday, t.tm_hour);

    Serial.println("---------------------------------------------------------------------");
  }

  // Update NTP every 6 hours
  if ((unsigned long)(ticks - ntpUpdated) >= 6 * 60 * 60 * 1000)
  {
    updateNTP();
  }

  delay(100);
}

void print_sensors()
{
  sensors_event_t event;
  bool noTemp = false;
  bool noHumidity = false;
  bool showMinMax = loops > 0 && !(loops % 5);
  int CO2;
  float t = 0.0, h = 0.0;

  Serial.println("print_sensors() [" + String(millis()) + "]");

  lastCO2 = CO2 = mhz19.getCO2();
  Serial.print("    CO2 (ppm): ");
  Serial.println(CO2);

  if (CO2 <= 0 || mhz19.errorCode != RESULT_OK)
  {
    co2Failures++;
    Serial.println("    MH-Z19B failure " + String(co2Failures) + " errorCode = " + String(mhz19.errorCode));
    mhz19.verify();
    if (mhz19.errorCode == RESULT_OK)
    {
      co2Failures = 0;
      CO2 = mhz19.getCO2();
      Serial.println("    MH-Z19B recovered");
    }
    else
    {
      if (co2Failures >= 3)
      {
        co2Failures = 0;
        Serial.println("    Preforming a reset on CO2 sensor - error_code = " + String(mhz19.errorCode));
        mhz19.recoveryReset();
        delay(5000);
      }
      return;
    }
  }

  if (CO2 >= CO2_MIN && CO2 <= CO2_MAX)
  {
    minCO2 = min(minCO2, CO2);
    maxCO2 = max(maxCO2, CO2);
  }

  Serial.print("    MH-Z19B Temperature (C): ");
  Serial.println(String(mhz19.getTemperature()));
  Serial.print("    Min CO2: ");
  Serial.println(minCO2);
  Serial.print("    Max CO2: ");
  Serial.println(maxCO2);

  // Get temperature event and print its value
  dht.temperature().getEvent(&event);
  t = event.temperature;
  Serial.println("    T: " + String(t) + " C / " + String(c_to_f(t)) + " F" );

  noTemp = isnan(t);
  if (noTemp)
  {
    Serial.println(F("    Error reading temperature!"));
    lcd.setCursor(0, 0);
    lcd.print("Error reading temperature            ");
  }
  else
  {
#ifdef DISPLAY_CELSIUS
    char c_or_f = 'C';
#else
    char c_or_f = 'F';
#endif

    lastTemperature = t;
    minTemp = min(minTemp, t);
    maxTemp = max(maxTemp, t);
    String msg;
    if (!showMinMax)
    {
      msg = "T: " + String(convert_temp(t)) + c_or_f + "  CO2:  ";
    }
    else
    {
      if (showTempMinMax)
      {
        msg = String(convert_temp(minTemp)) + c_or_f + " / " + String(convert_temp(maxTemp)) + c_or_f + "          ";
      }
      else
      {
        msg = String("CO2 Min: " + String(minCO2) + "                  ");
      }
    }

    lcd.setCursor(0, 0);
    lcd.print(msg);
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  h = event.relative_humidity;
  Serial.println("    H: " + String(h) + "%");

  noHumidity = isnan(h);
  if (noHumidity)
  {
    Serial.println(F("    Error reading humidity!"));
    lcd.setCursor(0, 0);
    lcd.print("Error reading humidity                    ");
  }
  else
  {
    lastHumidity = h;
    minHumidity = min(minHumidity, h);
    maxHumidity = max(maxHumidity, h);

    String msg;
    if (!showMinMax)
    {
      msg = "H: " + String(h) + "%  " + String(CO2) + "      ";
    }
    else
    {
      if (showTempMinMax)
      {
        msg = String(minHumidity) + "% / " + String(maxHumidity) + "%            ";
      }
      else
      {
        msg = String("CO2 Max: " + String(maxCO2) + "                   ");
      }

      showTempMinMax = !showTempMinMax;
    }

    lcd.setCursor(0, 1);
    lcd.print(msg);

    lastSensorTimestamp = time(NULL);
    strcpy(lastSensorLocaltime, localTime());

#ifdef UDP_HOST_IP
    char buf[1024];
    to_xml(t, h, CO2, lastSensorTimestamp, buf, sizeof(buf));
    udp.print(String(buf) + "\n");
#endif
  }

  loops += 1;
  if (loops >= monitoringEndLoops)
  {
    // Reset the min/max values and loop count
    if (!noTemp)
    {
      minTemp = maxTemp = t;
    }
    else
    {
      minTemp = NO_DATA_HIGH;
      maxTemp = NO_DATA_LOW;
    }

    if (!noHumidity)
    {
      minHumidity = maxHumidity = h;
    }
    else
    {
      minHumidity = NO_DATA_HIGH;
      maxHumidity = NO_DATA_LOW;
    }

    minCO2 = CO2_MAX;
    maxCO2 = CO2_MIN;

    loops = 0;
  }

  Serial.println("print_sensors() complete [" + String(millis()) + "]");
}

void move_pointer(int wday, int hour)
{
  Serial.println("move_pointer(" + String(wday) + ", " + String(hour) + ") [" + String(millis()) + "]");

  if (wday < 0 || wday > 6)
    wday = 0;
  if (hour < 0 || hour > 23)
    hour = 0;

  int left = dayPos[wday];
  int right = dayPos[wday + 1];
  float divsPerHour = (left - right) / 24.0;

  //Serial.print(("wday = " + String(wday) + "\n").c_str());
  //Serial.print((String(left) + " , " + String(right) + "\n").c_str());

  int pos = int(left - (hour * divsPerHour));
  Serial.println(("    " + String(localTime()) + ": Hour = " + String(hour) + " -> Pos = " + String(pos) + " [" + String(left) + " , " + String(right) + "]").c_str());
  servo.write(pos);

  Serial.println("move_pointer() complete [" + String(millis()) + "]");
}

void updateNTP()
{
  Serial.println("Updating time from NTP at [" + String(millis()) + "]");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  ntpUpdated = millis();
  Serial.println("NTP date = " + String(localTime()) + " [" + String(millis()) + "]");

  lcd.setCursor(0, 0);
  lcd.print("Updated NTP Time        ");
  lcd.setCursor(0, 1);
  lcd.print(String(localTime()));
}

char* localTime()
{
  memset(localTimeBuffer, 0, sizeof(localTimeBuffer));
  struct tm t;
  if (!getLocalTime(&t)) {
    Serial.println("Failed to obtain time");
    updateNTP();
    return localTimeBuffer;
  }

  strftime(localTimeBuffer, sizeof(localTimeBuffer), "%c", &t);
  return localTimeBuffer;
}

float convert_temp(float c)
{
#ifdef DISPLAY_CELSIUS
  return c;
#else
  return c_to_f(c);
#endif
}

float c_to_f(float c)
{
  return (c * 9.0 / 5.0) + 32.0;
}

void(* resetFunc)(void) = 0;

void to_xml(float temperature, float humidity, int co2, int lastSensorTimestamp, char *buf, int len)
{
  snprintf(buf, len - 1, "<xml millis=\"%d\" ts=\"%d\"><temp_f>%0.2f</temp_f><temp_c>%0.2f</temp_c><humidity>%0.2f</humidity><co2>%d</co2></xml>", millis(), lastSensorTimestamp, c_to_f(temperature), temperature, humidity, co2);
}

void to_json(float temperature, float humidity, int co2, int lastSensorTimestamp, char *buf, int len)
{
  snprintf(buf, len - 1, "{\"temp_f\": %0.2f, \"temp_c\": %0.2f, \"humidity\": %0.2f, \"co2\": %d, \"timestamp\": %d, \"millis\": %d, \"timestamp_str\": \"%s\"}", c_to_f(temperature), temperature, humidity, co2, lastSensorTimestamp, millis(), lastSensorLocaltime);
}

void httpRootHandler(AsyncWebServerRequest *request)
{
  Serial.println("HTTP request from " + request->client()->remoteIP().toString() + ": / [" + String(millis()) + "]");

  const char *html = R"(
<html>
   <head>
        <meta charset='UTF-8'>
        <meta name='viewport' content='width=device-width, initial-scale=1.0'>
        <title>Dayclock - %0.2f° %s / %d ppm</title>
        <script>
            setInterval(updateValues, 5000);

            function updateValues() {
                var use_celsius = %s;
                var xmlhttp = new XMLHttpRequest();
                xmlhttp.onreadystatechange = function() {
                    if (this.readyState == 4 && this.status == 200) {
                        var data = JSON.parse(this.responseText);
                        if (data.temp_f)
                            document.getElementById('temp_f_val').innerHTML = data.temp_f;
                        if (data.temp_c)
                            document.getElementById('temp_c_val').innerHTML = data.temp_c;
                        if (data.humidity)
                            document.getElementById('humidity_val').innerHTML = data.humidity;
                        if (data.co2)
                            document.getElementById('co2_val').innerHTML = data.co2;
                        if (data.timestamp)
                            document.getElementById('timestamp_val').innerHTML = data.timestamp_str;

                        document.title = 'Dayclock - ';
                        if (use_celsius)
                        {
                            document.title += document.getElementById('temp_c_val').innerHTML + '° C';
                        }
                        else
                        {
                            document.title += document.getElementById('temp_f_val').innerHTML + '° F';
                        }
                        document.title += ' / ' + document.getElementById('co2_val').innerHTML + ' ppm';
                    }
                };
                xmlhttp.open("GET", '/json', true);
                xmlhttp.send();
            }
        </script>
        <noscript>
            <meta http-equiv='refresh' content='5'/>
        </noscript>
        <style>
          dl { border: 3px double #ccc;  padding: 0.5em; width: 320px; }
          dt { float: left; clear: left; width: 100px; text-align: right; font-weight: bold; color: green; }
          dt::after { content: ':'; }
          dd { margin: 0 0 0 110px; padding: 0 0 0.5em 0; }
          %s
        </style>
  </head>
  <body>
    <dl>
      <span id='temp_f'><dt>Temperature</dt><dd><span id='temp_f_val'>%0.2f</span>&deg; F</dd></span>
      <span id='temp_c'><dt>Temperature</dt><dd><span id='temp_c_val'>%0.2f</span>&deg; C</dd></span>
      <span id='humidity'><dt>Humidity</dt><dd><span id='humidity_val'>%0.1f</span>%%</dd></span>
      <span id='co2'><dt>CO2</dt><dd><span id='co2_val'>%d</span> ppm</dd></span>
      <span id='timestamp'><dt>Updated</dt><dd><span id='timestamp_val'>%s</span></dd></span>
    </dl>
  </body>
</html>
)";

#ifdef DISPLAY_CELSIUS
  char *tempCSS = "#temp_f { display: none; }";
  float displayTemp = lastTemperature;
  char *tempUnit = "C";
#else
  char *tempCSS = "#temp_c { display: none; }";
  float displayTemp = c_to_f(lastTemperature);
  char *tempUnit = "F";
#endif

  char buf[4096];
  // Display Temperature, Temperature Unit, CO2, use_celsius - "true" (for C) or "false" (for F), temp_css, F temp, C temp, Humidity, CO2, Last Update Timestamp
  snprintf(buf, sizeof(buf) - 1, html, displayTemp, tempUnit, lastCO2,      // HTML Title (temperature float, temperature unit string, CO2 integer)
                                       tempUnit == "C" ? "true" : "false",  // use_celsius: Celsius = "true", F = "false" string
                                       tempCSS,                             // CSS string
                                       c_to_f(lastTemperature), lastTemperature, lastHumidity, lastCO2, lastSensorLocaltime);
  
  AsyncWebServerResponse *response = request->beginResponse(200, "text/html", buf);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
  
  Serial.println("HTTP request complete: / [" + String(millis()) + "]");
}

void httpXMLHandler(AsyncWebServerRequest *request)
{
  Serial.println("HTTP request from " + request->client()->remoteIP().toString() + ": /xml [" + String(millis()) + "]");

  char buf[1024];
  to_xml(lastTemperature, lastHumidity, lastCO2, lastSensorTimestamp, buf, sizeof(buf));

  AsyncWebServerResponse *response = request->beginResponse(200, "application/xml", buf);
  response->addHeader("Access-Control-Allow-Origin", "*");
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
    
  Serial.println("HTTP request complete: /xml [" + String(millis()) + "]");
}

void httpJSONHandler(AsyncWebServerRequest *request)
{
  Serial.println("HTTP request from " + request->client()->remoteIP().toString() + ": /json [" + String(millis()) + "]");

  char buf[1024];
  to_json(lastTemperature, lastHumidity, lastCO2, lastSensorTimestamp, buf, sizeof(buf));

  AsyncWebServerResponse *response = request->beginResponse(200, "application/json", buf);
  response->addHeader("Access-Control-Allow-Origin", "*");
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);

  Serial.println("HTTP request complete: /json [" + String(millis()) + "]");
}

void httpFaviconHandler(AsyncWebServerRequest *request)
{
  const uint8_t iconData[283] PROGMEM = {
  0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D,
  0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10,
  0x08, 0x04, 0x00, 0x00, 0x00, 0xB5, 0xFA, 0x37, 0xEA, 0x00, 0x00, 0x00,
  0x07, 0x74, 0x49, 0x4D, 0x45, 0x07, 0xE4, 0x05, 0x11, 0x14, 0x1E, 0x09,
  0xD7, 0xF6, 0xC0, 0x69, 0x00, 0x00, 0x00, 0x09, 0x70, 0x48, 0x59, 0x73,
  0x00, 0x00, 0x0E, 0xC3, 0x00, 0x00, 0x0E, 0xC3, 0x01, 0xC7, 0x6F, 0xA8,
  0x64, 0x00, 0x00, 0x00, 0x04, 0x67, 0x41, 0x4D, 0x41, 0x00, 0x00, 0xB1,
  0x8F, 0x0B, 0xFC, 0x61, 0x05, 0x00, 0x00, 0x00, 0xAA, 0x49, 0x44, 0x41,
  0x54, 0x78, 0xDA, 0xCD, 0x51, 0xD1, 0x0A, 0x82, 0x40, 0x10, 0x9C, 0x55,
  0xF3, 0x12, 0xD3, 0xEC, 0xA9, 0x8F, 0x28, 0xEA, 0xFF, 0x7F, 0x22, 0x22,
  0xA8, 0xE8, 0x07, 0xEA, 0x59, 0x23, 0x23, 0x4A, 0xA7, 0xD1, 0x20, 0x0B,
  0xA2, 0xB7, 0xA0, 0x3D, 0x76, 0x6F, 0x6E, 0x77, 0x18, 0x66, 0x39, 0x23,
  0xBE, 0x87, 0x87, 0x9F, 0x13, 0x82, 0x0E, 0x2E, 0xE8, 0x10, 0x83, 0x38,
  0x61, 0x6A, 0x5D, 0xD7, 0x1E, 0x26, 0x97, 0x4C, 0x11, 0x6A, 0x58, 0xC3,
  0xE0, 0xE3, 0x2A, 0x54, 0x60, 0x6E, 0x4F, 0xC2, 0x8E, 0x7D, 0x49, 0xC5,
  0xC8, 0xD4, 0x3A, 0x32, 0xB1, 0x82, 0x25, 0x4A, 0x9C, 0x31, 0xB1, 0x96,
  0xB0, 0x62, 0x06, 0x87, 0x08, 0xE9, 0x8B, 0x70, 0xCE, 0x8B, 0x28, 0x05,
  0x66, 0x26, 0x0F, 0x23, 0x09, 0x8F, 0xED, 0xDD, 0xDA, 0x50, 0xEF, 0x3D,
  0x93, 0x66, 0x8B, 0x0D, 0x29, 0xF1, 0x4F, 0x11, 0x2A, 0xB7, 0x0C, 0x1C,
  0x2A, 0x29, 0xAC, 0xD9, 0x93, 0x31, 0xD3, 0xA9, 0xDB, 0xF4, 0x54, 0x07,
  0xF2, 0x15, 0xC1, 0x0E, 0x6C, 0x98, 0xBE, 0xC6, 0x95, 0xEE, 0x9B, 0x06,
  0x9E, 0x70, 0xD0, 0x62, 0x68, 0x1F, 0xFB, 0x83, 0xBF, 0xB8, 0x03, 0x49,
  0x8E, 0x36, 0xD9, 0x7F, 0x4C, 0x9D, 0x98, 0x00, 0x00, 0x00, 0x00, 0x49,
  0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82
};
  AsyncWebServerResponse *response = request->beginResponse_P(200, "image/x-icon", iconData, sizeof(iconData));
  request->send(response);
}

void httpUpdateHandler(AsyncWebServerRequest *request)
{
  char* html = "<form method='POST' action='/do_update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
  request->send(200, "text/html", html);
}

void httpDoUpdateHandler(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) 
{
  if (!index)
  {
    Serial.println("Update Starting");
    updateContentLen = request->contentLength();

    lcd.setCursor(0, 0);
    lcd.print("Updating                           ");
    lcd.setCursor(0, 1);
    lcd.print(String(updateContentLen) + " bytes        ");
           
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ? U_SPIFFS : U_FLASH;
#ifdef ESP8266
    Update.runAsync(true);
    if (!Update.begin(updateContentLen, cmd)) 
    {
#else
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
    {
#endif
      updateContentLen = 0;
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) 
  {
    updateContentLen = 0;
    Update.printError(Serial);
#ifdef ESP8266
  } 
  else 
  {
    Serial.printf("Progress: %d%%\n", (Update.progress() * 100) / Update.size());
#endif
  }

  if (final) 
  {
    if (!Update.end(true))
    {
      updateContentLen = 0;
      Update.printError(Serial);

      char *html = "<html><head><meta http-equiv='refresh' content='5; url=/update'><title>Update Failed</title></head></html>";
      request->send(200, "text/html", html);
    } 
    else
    {
      char *html = "<html><head><meta http-equiv='refresh' content='10; url=/'><title>Rebooting</title></head><body>Rebooting...</body></html>";
      request->send(200, "text/html", html);
      update_reboot();
    }
  }
}

void update_reboot()
{
  lcd.setCursor(0, 0);
  lcd.print("Update Complete - Rebooting        ");
  lcd.setCursor(0, 1);
  lcd.print("                                   ");

  Serial.println("Update complete");
  Serial.flush();

  delay(3000);
  ESP.restart();
}

void update_print_progress(size_t prg, size_t sz) 
{
  int mapping[100];
  float pos = 180;
  for(int i=0; i<100; i++)
  {
    mapping[i] = int(pos);
    pos -= 1.8;
  }
  
  int progress = (prg * 100) / updateContentLen;
  Serial.printf("Progress: %d%%\n", progress);
  servo.write(mapping[progress]);
}
