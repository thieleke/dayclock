/*
 * Default Pinouts
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
const long  gmtOffset_sec      = -21600;          // Non-daylight saving time offset from UTC, in minutes (-18000 = US Eastern Standard Time, -21600 = US Central Standard Time, etc.)
const int   daylightOffset_sec = 3600;            // Number of minutes offset for daylight saving time (0 if daylight savings time if not applicable)

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
/* Constants that you'll definitely want to update */


#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <AsyncUDP.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <MHZ19.h>
#include <time.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h> 

unsigned long ntpUpdated = 0;
char localTimeBuffer[30];
WebServer webServer(80);
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

void setup() {
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
  lcd.setCursor(0,0);
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
  lcd.setCursor(0,0);
  lcd.print("Wi-Fi connected       ");
  lcd.setCursor(0, 1);
  lcd.print("                      ");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP().toString() + "               ");
  delay(2000);
  
  updateNTP();

  for(int i=0; i<8; i++)
  {
    lcd.setCursor(0,0);
    lcd.print(String(i) + " -> " + String(dayPos[i]) + "               ");
    servo.write(dayPos[i]);
    delay(1500);
  }

  for(int i=0; i<8; i++)
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
  if(udp.connect(IPAddress(UDP_HOST_IP), UDP_HOST_PORT)) {
      Serial.println("UDP connected");
      udp.onPacket([](AsyncUDPPacket packet) {
          Serial.print("UDP Packet Type: ");
          Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
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
  
  webServer.on("/", httpRootHandler);
  webServer.on("/xml", httpXMLHandler);
  webServer.on("/json", httpJSONHandler);
  webServer.enableCORS(true);
  webServer.begin();

  lcd.setCursor(0, 0);
  lcd.print("Starting...             ");
}

void loop() 
{  
  unsigned long ticks = millis();
  webServer.handleClient();

  if((unsigned long)(ticks - sensorDelayTicks) >= delayMS)
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
  if((unsigned long)(ticks - ntpUpdated) >= 6 * 60 * 60 * 1000)
  {
    updateNTP();
  }

  webServer.handleClient();

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

  if(CO2 <= 0 || mhz19.errorCode != RESULT_OK)
  {
    co2Failures++;
    Serial.println("    MH-Z19B failure " + String(co2Failures) + " errorCode = " + String(mhz19.errorCode));
    mhz19.verify();
    if(mhz19.errorCode == RESULT_OK)
    {
      co2Failures = 0;
      CO2 = mhz19.getCO2();
      Serial.println("    MH-Z19B recovered");
    }
    else
    {
      if(co2Failures >= 3)
      {
        co2Failures = 0;
        Serial.println("    Preforming a reset on CO2 sensor - error_code = " + String(mhz19.errorCode));
        mhz19.recoveryReset();
        delay(5000);
      }
      return;
    }
  } 

  if(CO2 >= CO2_MIN && CO2 <= CO2_MAX)
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
    if(!showMinMax)
    {
      msg = "T: " + String(convert_temp(t)) + c_or_f +"  CO2:  ";
    }
    else
    {
      if(showTempMinMax)
      {
        msg = String(convert_temp(minTemp)) + c_or_f + " / " + String(convert_temp(maxTemp)) + c_or_f + "          ";
      }
      else
      {
        msg = String("CO2 Min: " + String(minCO2) + "                  ");
      }      
    }

    lcd.setCursor(0,0);
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
    if(!showMinMax)
    {
      msg = "H: " + String(h) + "%  " + String(CO2) + "      ";
    }
    else
    {     
      if(showTempMinMax)
      {
        msg = String(minHumidity) + "% / " + String(maxHumidity) + "%            ";
      }
      else
      {
        msg = String("CO2 Max: " + String(maxCO2) + "                   ");
      }

      showTempMinMax = !showTempMinMax;
    }
    
    lcd.setCursor(0,1);
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
  if(loops >= monitoringEndLoops)
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
}

void move_pointer(int wday, int hour)
{
  Serial.println("move_pointer(" + String(wday) + ", " + String(hour) + ") [" + String(millis()) + "]");
  
  if(wday < 0 || wday > 6)
    wday = 0;
  if(hour < 0 || hour > 23)
    hour = 0;
    
  int left = dayPos[wday];
  int right = dayPos[wday + 1];
  float divsPerHour = (left - right) / 24.0;
  
  //Serial.print(("wday = " + String(wday) + "\n").c_str());
  //Serial.print((String(left) + " , " + String(right) + "\n").c_str());
  
  int pos = int(left - (hour * divsPerHour));
  Serial.println(("    " + String(localTime()) + ": Hour = " + String(hour) + " -> Pos = " + String(pos) + " [" + String(left) + " , " + String(right) + "]").c_str());
  servo.write(pos);
}

void updateNTP()
{
    Serial.println("Updating time from NTP at [" + String(millis()) + "]");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    ntpUpdated = millis();
    Serial.println("NTP date = " + String(localTime()) + " [" + String(millis()) + "]");

    lcd.setCursor(0,0);
    lcd.print("Updated NTP Time        ");
    lcd.setCursor(0, 1);
    lcd.print(String(localTime()));
}

char* localTime()
{
  memset(localTimeBuffer, 0, sizeof(localTimeBuffer));
  struct tm t;
  if(!getLocalTime(&t)){
    Serial.println("Failed to obtain time");
    updateNTP();
    return localTimeBuffer;
  }

  strftime(localTimeBuffer, sizeof(localTimeBuffer), "%c", &t);
  return localTimeBuffer;
}

void to_xml(float temperature, float humidity, int co2, int lastSensorTimestamp, char *buf, int len)
{
  snprintf(buf, len - 1, "<xml millis=\"%d\" ts=\"%d\"><temp_f>%0.2f</temp_f><temp_c>%0.2f</temp_c><humidity>%0.2f</humidity><co2>%d</co2></xml>", millis(), lastSensorTimestamp, c_to_f(temperature), temperature, humidity, co2);
}

void to_json(float temperature, float humidity, int co2, int lastSensorTimestamp, char *buf, int len)
{
  snprintf(buf, len - 1, "{\"temp_f\": %0.2f, \"temp_c\": %0.2f, \"humidity\": %0.2f, \"co2\": %d, \"timestamp\": %d, \"millis\": %d, \"timestamp_str\": \"%s\"}", c_to_f(temperature), temperature, humidity, co2, lastSensorTimestamp, millis(), lastSensorLocaltime); 
}

void httpRootHandler()
{
  Serial.println("HTTP request: root");

   const char *html = R"(
<html>
   <head>
        <meta charset='UTF-8'>
        <meta name='viewport' content='width=device-width, initial-scale=1.0'>
        <title>Dayclock</title>
        <script>
            setInterval(updateValues, 5000);

            function updateValues() {
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
  char *temp_css = "#temp_f { display: none; }";
#else
  char *temp_css = "#temp_c { display: none; }";
#endif

  char buf[4096];
  snprintf(buf, sizeof(buf) - 1, html, temp_css, c_to_f(lastTemperature), lastTemperature, lastHumidity, lastCO2, lastSensorLocaltime);
  webServer.send(200, "text/html", buf);
  
  Serial.println("HTTP request complete: root");
}

void httpXMLHandler()
{
  Serial.println("HTTP request: xml");

  char buf[1024];
  to_xml(lastTemperature, lastHumidity, lastCO2, lastSensorTimestamp, buf, sizeof(buf));
  webServer.send(200, "text/xml", buf);

  Serial.println("HTTP request complete: xml");
}

void httpJSONHandler()
{
  Serial.println("HTTP request: json");
  
  char buf[1024];
  to_json(lastTemperature, lastHumidity, lastCO2, lastSensorTimestamp, buf, sizeof(buf));
  webServer.send(200, "application/json", buf);

  Serial.println("HTTP request complete: json");
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
  return (c * 9.0/5.0) + 32.0;
}

void(* resetFunc)(void) = 0;
