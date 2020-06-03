#ifndef DAYCLOCK_CONFIG_H
#define DAYCLOCK_CONFIG_H

//#define DISPLAY_CELSIUS                         // Uncomment if you want the LCD and HTTP displays to default to Celsius temperature instead of Fahrenheit
#define UDP_HOST_IP            192,168,0,1        // IP address, comma separated (eg: 192,168,0,1), if you want to receive UDP push messages.
#define UDP_HOST_PORT          1234               // UDP port
const char *ssid               = "YOUR_SSID";     // Your WiFi SSID
const char *password           = "PASSWORD";      // Your WiFi password
const char* ntpServer          = "pool.ntp.org";  // There are also region-specific addresses such as "us.pool.ntp.org"
const long  gmtOffset_sec      = -21600;          // Non-daylight saving time offset from UTC, in seconds (-18000 = US Eastern Standard Time, -21600 = US Central Standard Time, etc.)
const int   daylightOffset_sec = 3600;            // Number of seconds offset for daylight saving time (0 if daylight savings time if not applicable)

#define SERVO_PIN  13
#define DHT_PIN    14
#define LED_BUILTIN 2

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
#define MHZ19_BAUDRATE 9600   // Device to MH-Z19 Serial baudrate (should not be changed)
#define CO2_MIN  350
#define CO2_MAX  4000

// Defaults to I2C address 0x27 and 16x2 display
#define LCD_I2C_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

#define HISTORY_COUNT         3600    // Set to 0 to disable
#define HISTORY_INTERVAL_SEC  60


#endif  // DAYCLOCK_CONFIG_H
