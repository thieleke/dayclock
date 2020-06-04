/*
   Default Pinouts
 *    * Servo:  GPIO Pin 13
 *    * DHT: 5V, GND, GPIO Pin 14
 *    * LCD: 5V, GND, SDA -> I2C SDA, SCL -> IC2 SCL
 *    * MH-Z19B: RX -> TX2, TX -> RX2
*/

// NOTE: See Config.h for constants that you'll definitely want to update
#include "Config.h"

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

#define NO_DATA_LOW -99999
#define NO_DATA_HIGH 99999

unsigned long ntpUpdatedTicks = 0;
unsigned long wifiConnectedTicks = 0;
char localTimeBuffer[30];
#ifdef UDP_HOST_IP
char udpBuffer[256];
#endif
AsyncWebServer webServer(80);
AsyncUDP udp;
Servo servo;
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
DHT_Unified dht(DHT_PIN, DHT_TYPE);
MHZ19 mhz19;
HardwareSerial mhz19Serial(MHZ19_SERIAL_PORT);

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
time_t lastSensorTimestamp = 0;
char  lastSensorLocaltime[30];
size_t updateContentLen = 0;
bool inHistoryHandler = false; 

#if HISTORY_COUNT
struct history_t
{
  time_t timestamp;
  float temperature;
  float humidity;
  unsigned int co2;  
};

history_t historyVals[HISTORY_COUNT];
volatile int historyPos = HISTORY_COUNT - 1;
volatile bool in_add_history = false;
#endif

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

  // Built-in LED is used to indicate Wi-Fi connection status
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Connect to WiFi
  if(wifi_connect() == false)
  {
    lcd_print("Wi-Fi unavailable", 0);
    lcd_print("", 1);
    wifiConnectedTicks = 0;
  }  
  delay(2000);

  updateNTP();

#ifdef DEMO
  for (int i = 0; i < 8; i++)
  {
    lcd_print(String(i) + " -> " + String(dayPos[i]), 0);
    servo.write(dayPos[i]);
    yield();
    delay(1500);
  }
#endif

  for (int i = 0; i < 8; i++)
  {
    Serial.printf("%d -> %d\n", i, dayPos[i]);
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
  mhz19Serial.begin(MHZ19_BAUDRATE);
  mhz19.begin(mhz19Serial);
  mhz19.autoCalibration();

  // Set delay between sensor readings based on sensor details
  delayMS = 5000; //(sensor.min_delay / 1000) * 2;
  monitoringEndLoops = (24 * 60 * 60) / (delayMS / 1000);

#ifdef UDP_HOST_IP
  // nc -u -l 1234
  if (udp.connect(IPAddress(UDP_HOST_IP), UDP_HOST_PORT)) 
  {
    Serial.println("UDP initialized");
    udp.onPacket([](AsyncUDPPacket packet) 
    {
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
      packet.printf("Got %u bytes of data\n", packet.length());
    });

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
#if HISTORY_COUNT
  webServer.on("/chart",       HTTP_GET,  httpChartHandler);
  webServer.on("/history",     HTTP_GET,  httpHistoryHandler);
  webServer.on("/chart.png",   HTTP_GET,  httpChartIconHandler);
#endif
  webServer.on("/update",      HTTP_GET,  httpUpdateHandler);
  webServer.on("/do_update",   HTTP_POST, [](AsyncWebServerRequest * request) {},
                                          [](AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t *data, size_t len, bool final) {
                                               httpDoUpdateHandler(request, filename, index, data, len, final); });
  webServer.begin();

#ifdef ESP32
  Update.onProgress(update_print_progress);
#endif

#if HISTORY_COUNT
  memset(historyVals, 0, sizeof(historyVals));
#endif

  lcd_print("Starting...", 0);
  lcd_print("", 1);
}

void loop()
{
  // Don't loop while performing an update
  if (updateContentLen > 0)
    return;

  const unsigned long ticks = millis();

  if ((unsigned long)(ticks - sensorDelayTicks) >= delayMS)
  {
    sensorDelayTicks = ticks;

    // Update the LCD with temperature, humidity, and CO2
    update_sensors();

    struct tm t;
    getLocalTime(&t);
    move_pointer(t.tm_wday, t.tm_hour);
    Serial.println("---------------------------------------------------------------------");
  }

  // Ensure that we're connected to WiFi (if available) every 5 minutes
  if(wifiConnectedTicks > 0 && (unsigned long)(ticks - wifiConnectedTicks) >= 5 * 60 * 1000)
  {
    if(wifi_connect() == false)
    {
      // Try again in 5 minutes
      Serial.println("WiFi reconnected failed - retrying in 5 minutes");
      wifiConnectedTicks = ticks + (5 * 60 * 1000);
    }
  }
  
  // Update NTP every 6 hours
  if ((unsigned long)(ticks - ntpUpdatedTicks) >= 6 * 60 * 60 * 1000)
  {
    updateNTP();
  }
}


void update_sensors()
{
  sensors_event_t event;
  bool noTemp = false;
  bool noHumidity = false;
  bool noCO2 = false;
  bool showMinMax = loops > 0 && !(loops % 5);
  int CO2;
  float t = 0.0, h = 0.0;

  Serial.printf("update_sensors() [%lu]\n", millis());

  CO2 = mhz19.getCO2();
  yield();
  Serial.printf("    CO2 (ppm): %d\n", CO2);

  if (CO2 <= 0 || mhz19.errorCode != RESULT_OK)
  {
    co2Failures++;
    Serial.printf("    MH-Z19B failure %d, errorCode = %d\n", co2Failures, mhz19.errorCode);
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
        Serial.printf("    Preforming a reset on CO2 sensor - error_code = %d [%lu]", mhz19.errorCode, millis());
        mhz19.recoveryReset();
        yield();
        delay(1000);
      }
    }
  }

  if (CO2 >= CO2_MIN && CO2 <= CO2_MAX)
  {
    lastCO2 = CO2;
    minCO2 = min(minCO2, CO2);
    maxCO2 = max(maxCO2, CO2);
  }
  else
  {
    noCO2 = true;
  }
  
  //Serial.print("    MH-Z19B Temperature (C): ");
  //Serial.println(String(mhz19.getTemperature()));
  Serial.print("    Min CO2: ");
  Serial.println(minCO2);
  Serial.print("    Max CO2: ");
  Serial.println(maxCO2);

  // Get temperature event and print its value
  dht.temperature().getEvent(&event);
  t = event.temperature;
  yield();  
  Serial.printf("    T: %0.2f C / %0.2f F\n", t, c_to_f(t));

  noTemp = isnan(t) || t < -40 || t > 125;
  if (noTemp)
  {
    Serial.println("    Error reading temperature!");   
    lcd_print("Error reading temperature", 0);
  }
  else
  {
#ifdef DISPLAY_CELSIUS
    const char c_or_f = 'C';
#else
    const char c_or_f = 'F';
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

    lcd_print(msg, 0);
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  h = event.relative_humidity;
  yield();
  Serial.printf("    H: %0.2f%%\n", h);

  noHumidity = isnan(h) || h < 0 || h > 100;
  if (noHumidity)
  {
    Serial.println("    Error reading humidity");
    lcd_print("Error reading humidity", 0);
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

    lcd_print(msg, 1);
  }

  // Ignore errored sensor readings and try later
  if(!noTemp && !noHumidity && !noCO2)
  {
    lastSensorTimestamp = time(NULL);
    strncpy(lastSensorLocaltime, localTime(), sizeof(lastSensorLocaltime) - 1);
  
#if HISTORY_COUNT
    history_t *history = get_current_history();
    if(lastSensorTimestamp - history->timestamp >= HISTORY_INTERVAL_SEC)
    {
      add_history(lastSensorTimestamp, lastTemperature, lastHumidity, lastCO2);
    }
#endif    
    
#ifdef UDP_HOST_IP
    to_xml(t, h, CO2, lastSensorTimestamp, udpBuffer, sizeof(udpBuffer));
    udp.print(udpBuffer);
    udp.print("\n");
#endif
  }

  digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED ? HIGH : LOW);

  if (++loops >= monitoringEndLoops)
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

    if (!noCO2)
    {
      minCO2 = CO2_MAX;
      maxCO2 = CO2_MIN;
    }
    
    loops = 0;
  }

  Serial.printf("update_sensors() complete [%lu]\n", millis());
}

void move_pointer(int wday, int hour)
{
  Serial.printf("move_pointer(%d, %d) [%lu]\n", wday, hour, millis());

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
  //Serial.printf("    %s: Hour = %d -> Pos = %d [%d, %d]\n", localTime(), hour, pos, left, right);
  servo.write(pos);

  Serial.printf("move_pointer() complete [%lu]\n", millis());
}

void updateNTP()
{
  Serial.printf("Updating time from NTP at [%lu]\n", millis());
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  ntpUpdatedTicks = millis();
  Serial.printf("NTP date = %s [%lu]\n", localTime(), millis());

  lcd_print("Updated NTP Time", 0);
  lcd_print(String(localTime()), 1);
}
