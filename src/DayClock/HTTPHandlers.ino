void to_xml(float temperature, float humidity, int co2, int lastSensorTimestamp, char *buf, int len)
{
  snprintf(buf, len - 1, "<xml millis=\"%lu\" ts=\"%d\"><temp_f>%0.2f</temp_f><temp_c>%0.2f</temp_c><humidity>%0.2f</humidity><co2>%d</co2></xml>", millis(), lastSensorTimestamp, c_to_f(temperature), temperature, humidity, co2);
}

void to_json(float temperature, float humidity, int co2, int lastSensorTimestamp, char *buf, int len)
{
  snprintf(buf, len - 1, "{\"temp_f\": %0.2f, \"temp_c\": %0.2f, \"humidity\": %0.2f, \"co2\": %d, \"timestamp\": %d, \"millis\": %lu, \"timestamp_str\": \"%s\"}", c_to_f(temperature), temperature, humidity, co2, lastSensorTimestamp, millis(), lastSensorLocaltime);
}

void log_start_request(AsyncWebServerRequest *request, const char *path)
{
  IPAddress addr = request->client()->remoteIP();
  Serial.printf("HTTP request from %u.%u.%u.%u: %s [%lu]\n", addr[0], addr[1], addr[2], addr[3], path, millis());
}

void log_end_request(const char *path)
{
  Serial.printf("HTTP request complete: %s [%lu]\n", path, millis());
}

void httpRootHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/");

  AsyncWebServerResponse *response = authenticationCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/ (missing authentication)");
    return;
  }

  static const char *html = R"(
<html>
   <head>
        <meta charset='UTF-8'>
        <meta name='viewport' content='width=device-width, initial-scale=1.0'>
        <title>Dayclock  %0.2f° %s / %d ppm</title>
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

                        document.title = 'Dayclock';
                        if (use_celsius)
                        {
                            document.title += "  " + document.getElementById('temp_c_val').innerHTML + '° C';
                        }
                        else
                        {
                            document.title += "  " + document.getElementById('temp_f_val').innerHTML + '° F';
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
      %s
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
  const char *tempCSS = "#temp_f { display: none; }";
  float displayTemp = lastTemperature;
  const char *tempUnit = "C";
#else
  const char *tempCSS = "#temp_c { display: none; }";
  float displayTemp = c_to_f(lastTemperature);
  const char *tempUnit = "F";
#endif

#if HISTORY_COUNT
  const char *chartHTML = "<a href='/chart' title='Chart' target='_chart'><img src='/chart.png' style='float: right;' width='32' height='32'></a>";
#else
  const char *chartHTML = "";
#endif

  char buf[4096];
  // Chart HTML, Display Temperature, Temperature Unit, CO2, use_celsius - "true" (for C) or "false" (for F), CSS, Chart HTML, F temp, C temp, Humidity, CO2, Last Update Timestamp
  snprintf(buf, sizeof(buf) - 1, html, displayTemp, tempUnit, lastCO2,                 // HTML Title (temperature float, temperature unit string, CO2 integer)
                                       strcmp(tempUnit, "C") == 0 ? "true" : "false",  // use_celsius: Celsius = "true", F = "false" string
                                       tempCSS,                                        // CSS string
                                       chartHTML,                                      // Chart HTML link
                                       c_to_f(lastTemperature), lastTemperature, lastHumidity, lastCO2, lastSensorLocaltime);

  response = request->beginResponse(200, "text/html", buf);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
  
  log_end_request("/");
}

void httpXMLHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/xml");

  AsyncWebServerResponse *response = authenticationCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/xml (missing authentication)");
    return;
  }

  char buf[1024];
  to_xml(lastTemperature, lastHumidity, lastCO2, lastSensorTimestamp, buf, sizeof(buf));

  response = request->beginResponse(200, "application/xml", buf);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
    
  log_end_request("/xml");
}

void httpJSONHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/json");

  AsyncWebServerResponse *response = authenticationCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/json (missing authentication)");
    return;
  }

  char buf[1024];
  to_json(lastTemperature, lastHumidity, lastCO2, lastSensorTimestamp, buf, sizeof(buf));

  response = request->beginResponse(200, "application/json", buf);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);

  log_end_request("/json");
}

#if HISTORY_COUNT
void httpChartHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/chart");

  AsyncWebServerResponse *response = authenticationCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/chart (missing authentication)");
    return;
  }

  static const char *html = R"(
<html>
<head>
    <title>Dayclock - History Chart</title>
    <meta charset='UTF-8'>
    <script src='https://cdnjs.cloudflare.com/ajax/libs/moment.js/2.18.1/moment.min.js'></script>
    <script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.9.3/Chart.min.js'></script>
    <style>
        canvas 
        {
            -moz-user-select: none;
            -webkit-user-select: none;
            -ms-user-select: none;
        }
    </style>
</head>
<body onload='getData()'>
    <div style='width:80%%;margin-left:auto;margin-right:auto;'>
        <canvas id='chart'></canvas>
    </div>
    <script>
        var data = null;

        function getData(do_async=true) {
            let xmlhttp = new XMLHttpRequest();
            xmlhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    data = JSON.parse(this.responseText);
                    updateChart();
                }
            }
            xmlhttp.open('GET', '/history', do_async);
            xmlhttp.send();
        }
        
        function getCO2Data() {
            let l = [];
            if(data !== null)
                data.forEach(item => { l.push({t: new Date(item[0] * 1000), y: item[4]}); });
            return l;
        }

        function getTemperatureData() {
            let l = []
            if(data !== null)
                data.forEach(item => { l.push({t: new Date(item[0] * 1000), y: item[%d]}); });
            return l;
        }
        
        function getHumidityData() {
            let l = []
            if(data !== null)
                data.forEach(item => { l.push({t: new Date(item[0] * 1000), y: item[3]}); });
            return l;
        }
        
        function updateChart() {
            if(data === null)
                console.log('updateChart - data is null');
                
            chart.data.datasets[0].data = getCO2Data();
            chart.data.datasets[1].data = getTemperatureData();
            chart.data.datasets[2].data = getHumidityData();
            chart.update();

            let i = Math.max(data.length, 1) - 1;
            document.title = "Dayclock " + data[i][%d] + "°%s / " + data[i][4] + " ppm";
        }

        var ctx = document.getElementById('chart').getContext('2d');
        var cfg = {
            data: {
                datasets: [{
                    label: 'CO2 (ppm)',
                    backgroundColor: '#4dc9f6',
                    borderColor: '#4dc9f6',
                    data: getCO2Data(),
                    type: 'line',
                    pointRadius: 0,
                    fill: false,
                    lineTension: 1,
                    cubicInterpolationMode: 'monotone',
                    borderWidth: 2,
                    yAxisID: 'y-axis-1',
                },
                {
                    label: 'Temperature (°%s)',
                    backgroundColor: '#f00000',
                    borderColor: '#f00000',
                    data: getTemperatureData(),
                    type: 'line',
                    pointRadius: 0,
                    fill: false,
                    lineTension: 1,
                    cubicInterpolationMode: 'monotone',
                    borderWidth: 2,
                    yAxisID: 'y-axis-2',
                },
                {
                    label: 'Humidity (%%)',
                    backgroundColor: '#42f5b3',
                    borderColor: '#42f5b3',
                    data: getHumidityData(),
                    type: 'line',
                    pointRadius: 0,
                    fill: false,
                    lineTension: 1,
                    cubicInterpolationMode: 'monotone',
                    borderWidth: 2,
                    yAxisID: 'y-axis-3',
                    hidden: true,
                }],
            },
            options: {
                responsive: true,
                animation: {
                    duration: 0
                },
                scales: {
                    xAxes: [{
                        type: 'time',
                        distribution: 'series',
                        offset: true,
                        ticks: {
                            major: {
                                enabled: true,
                                fontStyle: 'bold'
                            },
                            source: 'data',
                            autoSkip: true,
                            autoSkipPadding: 75,
                            maxRotation: 0,
                            sampleSize: 100
                        },
                        afterBuildTicks: function(scale, ticks) {
                            if(ticks === null)
                                return;
                            var majorUnit = scale._majorUnit;
                            var firstTick = ticks[0];
                            var i, ilen, val, tick, currMajor, lastMajor;

                            val = moment(ticks[0].value);
                            if (val.second() === 0) {
                                firstTick.major = true;
                            } else {
                                firstTick.major = false;
                            }
                            lastMajor = val.get(majorUnit);

                            for (i = 1, ilen = ticks.length; i < ilen; i++) {
                                tick = ticks[i];
                                val = moment(tick.value);
                                currMajor = val.get(majorUnit);
                                tick.major = currMajor !== lastMajor;
                                lastMajor = currMajor;
                            }
                            return ticks;
                        }
                    }],
                    yAxes: [{
                        gridLines: {
                            drawBorder: false
                        },
                        scaleLabel: {
                            display: true,
                            labelString: 'CO2 (ppm)'
                        },
                        position: 'left',
                        id: 'y-axis-1',
                        display: 'auto',
                        suggestedMin: 350,
                        suggestedMax: 4000
                    },
                    {
                        gridLines: {
                            drawBorder: false,
                            display: false
                        },
                        scaleLabel: {
                            display: true,
                            labelString: 'Temperature (°%s)'
                        },
                        position: 'right',
                        id: 'y-axis-2',
                        display: 'auto'
                    },
                    {
                        gridLines: {
                            drawBorder: false,
                            display: false
                        },
                        scaleLabel: {
                            display: true,
                            labelString: 'Humidity (%%)'
                        },
                        position: 'right',
                        id: 'y-axis-3',
                        display: 'auto'
                    }
                    ]
                },
                tooltips: {
                    intersect: false,
                    mode: 'index',
                    callbacks: {
                        label: function(tooltipItem, myData) {
                            var label = myData.datasets[tooltipItem.datasetIndex].label || '';
                            if (label) {
                                label += ': ';
                            }
                            label += tooltipItem.value;
                            return label;
                        }
                    }
                }
            }
        };

        var chart = new Chart(ctx, cfg);
        setInterval(getData, 30000);
    </script>
</body>
</html>
)";

#ifdef DISPLAY_CELSIUS
  int tempIndex = 2;
  const char *tempUnit = "C";
#else
  int tempIndex = 1;
  const char *tempUnit = "F";
#endif

  char buf[10000];
  snprintf(buf, sizeof(buf) - 1, html, tempIndex, tempIndex, tempUnit, tempUnit, tempUnit);
  
  response = request->beginResponse(200, "text/html", buf);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
  
  log_end_request("/chart");
}

volatile unsigned long _historyStartTicks = 0;
volatile static int _historyPos = -1;
volatile static int _historyChunkCount = 0;
volatile static bool _historyDone = false;

bool isHistoryBusy()
{
  // Unblock after 5 seconds
  if(_historyStartTicks != 0 && (unsigned long)(millis() - _historyStartTicks) >= 5000)
  {
    Serial.println("isHistoryBusy - unblocking");
    _historyStartTicks = 0;
  }
  
  return in_add_history || _historyStartTicks > 0;
}

bool setHistoryBusy(bool busy)
{
  if(busy && isHistoryBusy())
    return(false);

  if(busy)
  { 
    _historyStartTicks = millis();
  }
  else
  {
    _historyStartTicks = 0;
  }

  return(true);
}

void httpHistoryHandler(AsyncWebServerRequest *request)
{
  log_free_memory("httpHistoryHandler() start");
  log_start_request(request, "/history");

  AsyncWebServerResponse *response = authenticationCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/history (missing authentication)");
    return;
  }

  if(!setHistoryBusy(true))
  {
    Serial.println("httpHistoryHandler - busy");
    delay(100);
    request->redirect("/history?ms=" + String(millis())); 
    return;
  }
  
  time_t minTimestamp = 0;
  int minPos = 0;
  
  // Find the initial position based on the lowest timestamp
  for(int i=0; i<HISTORY_COUNT; i++)
  {
    history_t *h = get_history(i);
    if(h->timestamp == 0)
      break;

    if(minTimestamp == 0 || h->timestamp < minTimestamp)
    {
      minTimestamp = h->timestamp;
      minPos = i;
    }
  }

  _historyChunkCount = 0;
  _historyPos = minPos;
  _historyDone = false;

  response = request->beginChunkedResponse("application/json", get_history_data_callback);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);

  log_free_memory("httpHistoryHandler() end");
  log_end_request("/history");
}

size_t get_history_data_callback(uint8_t *buffer, size_t maxLen, size_t index)
{ 
  Serial.printf("get_history_data(%d, %d) - _historyDone = %s, _historyPos = %d  [%lu]\n", maxLen, index, _historyDone ? "true" : "false", _historyPos, millis());

  if(_historyDone)
  {
    setHistoryBusy(false);
    return 0;
  }

  history_t *h = get_history(_historyPos);

  const size_t maxItemSize = 38;
  const size_t maxCount = maxLen / maxItemSize;
  char outputBuf[(maxCount * maxItemSize) + 1];
  memset(outputBuf, 0, sizeof(outputBuf));
  size_t totalBytes = 0;

  for(size_t i=0; i<maxCount; i++)
  {    
    char tmp[maxItemSize + 1];
    const char prefix = index > 0 || i > 0 ? ',' : '[';
    
#if DH_SENSOR_TYPE == DHT_SENSOR_TYPE
    // Round lower accuracy sensors to one decimal digit
    snprintf(tmp, sizeof(tmp), "%c[%lu,%0.1f,%0.1f,%0.1f,%u]", prefix, h->timestamp, round_to_tenths(c_to_f(h->temperature)), round_to_tenths(h->temperature), round_to_tenths(h->humidity), h->co2);
#else
    snprintf(tmp, sizeof(tmp), "%c[%lu,%0.2f,%0.2f,%0.1f,%u]", prefix, h->timestamp, c_to_f(h->temperature), h->temperature, round_to_tenths(h->humidity), h->co2);
#endif

    strcat(outputBuf, tmp);
    totalBytes += strlen(tmp);

    _historyPos = get_next_index(_historyPos);
    h = get_history(_historyPos);
    if(h->timestamp == 0 || ++_historyChunkCount >= HISTORY_COUNT)
    {
       // Set _historyDone so that the next callback will return 0, ending the chunked request    
      Serial.printf("get_history_data() - done [%lu]\n", millis());   
      strcat(outputBuf, "]");
      totalBytes += 1;
      _historyDone = true;
      break;
    }
  }

  memcpy(buffer, outputBuf, totalBytes);

  Serial.printf("get_history_data() - returning %u bytes [%lu]\n", totalBytes, millis());
  return totalBytes;
}
#endif

static char csrf_token[32];
void httpUpdateHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/update");

  AsyncWebServerResponse *response = authenticationAdminCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/update (missing authentication)");
    return;
  }

  srand(millis());
  for(int i=0; i<32; i++)
  {
    csrf_token[i] = 'A' + (rand() % 26);
  }
  csrf_token[31] = '\0';
  
  char cookie[128];
  snprintf(cookie, sizeof(cookie), "CSRF_TOKEN=%s; HttpOnly; Max-Age=300; SameSite=Strict", csrf_token);
  
  const char *html = "<html><head><title>Firmware Update</title></head><body><form method='POST' action='/do_update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form></body></html>";
  response = request->beginResponse(200, "text/html", html);
  response->addHeader("Cache-Control", "no-cache");
  response->addHeader("Set-Cookie", cookie);
  request->send(response);

  log_end_request("/update");
}

void httpDoUpdateHandler(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) 
{
  log_start_request(request, "/do_update");

  if(request->hasHeader("Cookie") == false)
  {
    log_end_request("/do_update (missing CSRF_TOKEN cookie)");
    request->send(400);
    return;
  }
  
  AsyncWebHeader *cookie = request->getHeader("Cookie");
  char expectedCookie[64];
  snprintf(expectedCookie, sizeof(expectedCookie), "CSRF_TOKEN=%s", csrf_token);  
  if(cookie->value() != expectedCookie)
  {
    log_end_request("/do_update (invalid CSRF_TOKEN cookie)");
    request->send(400);
    return;    
  }
  
  if (!index)
  {
    Serial.println("Update Starting");
    updateContentLen = request->contentLength();
    
    lcd_print("Updating", 0);
    lcd_print(String(updateContentLen) + " bytes", 1);
           
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

      const char *html = "<html><head><meta http-equiv='refresh' content='5; url=/update'><title>Update Failed</title></head></html>";
      request->send(200, "text/html", html);
    } 
    else
    {
      const char *html = "<html><head><meta http-equiv='refresh' content='10; url=/'><title>Rebooting</title></head><body>Rebooting...</body></html>";
      request->send(200, "text/html", html);
      update_reboot();
    }
  }

  log_end_request("/do_update");
}

void update_reboot()
{
  lcd_print("Update Complete - Rebooting", 0);
  lcd_print("", 1);

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

void httpResetHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/reset");

  AsyncWebServerResponse *response = authenticationAdminCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/reset (missing authentication)");
    return;
  }

  const char *html = "<html><head><meta http-equiv='refresh' content='10; url=/'><title>Rebooting</title></head><body>Rebooting...</body></html>";
  request->send(200, "text/html", html);
  delay(1000);
  
  ESP.restart();
}

void httpCalibrateHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/calibrate");

  AsyncWebServerResponse *response = authenticationAdminCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/calibrate (missing authentication)");
    return;
  }
  
  mhz19.calibrate();
  const char *html = "<html><head><meta http-equiv='refresh' content='5; url=/'><title>Calibrating</title></head><body>Starting calibration...</body></html>";
  request->send(200, "text/html", html);
  
  log_end_request("/calibrate");
}

void httpAutoCalibrateHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/auto_calibrate");

  AsyncWebServerResponse *response = authenticationAdminCheck(request);
  if(response != NULL)
  {
    request->send(response);
    log_end_request("/auto_calibrate (missing authentication)");
    return;
  }

  bool new_abcStatus = mhz19.getABC() ? false : true;
  mhz19.autoCalibration(new_abcStatus);
  char buf[160];
  const char *msg =  mhz19.getABC() ? "Enabled" : "Disabled";
  snprintf(buf, sizeof(buf) - 1, 
           "<html><head><meta http-equiv='refresh' content='5; url=/'><title>Auto Calibrate %s</title></head><body>Auto Calibrate %s</body></html>", msg, msg);
  request->send(200, "text/html", buf);
  
  log_end_request("/auto_calibrate");
}



AsyncWebServerResponse* authenticationCheck(AsyncWebServerRequest *request)
{
  return(authenticationCheck(request, false));
}

AsyncWebServerResponse* authenticationAdminCheck(AsyncWebServerRequest *request)
{
  return(authenticationCheck(request, true));
}

AsyncWebServerResponse* authenticationCheck(AsyncWebServerRequest *request, bool admin)
{
  const char *authString = admin == true ? basicAuthAdminString : basicAuthString;
  const char *realm      = admin == true ? "DayClock Admin" : "DayClock";
  
  // If the authentication string isn't set or is empty, skip
  if(authString == NULL || strcmp(authString, "") == 0)
    return NULL;

  if(request->hasHeader("Authorization"))
  {    
    if(request->header("Authorization") == String("Basic ") + authString)
    {
      Serial.println("Authentication valid");
      return NULL;
    }

    Serial.printf("Authentication '%s' is invalid (authString = %s)\n", request->header("Authorization").c_str(), authString);
  }

  // Return the 401 response
  AsyncWebServerResponse *response = request->beginResponse(401);
  response->addHeader("WWW-Authenticate", "Basic realm=\"" + String(realm) + "\"");
  return response;
}

void httpFaviconHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/favicon.ico");
  
  static const uint8_t iconData[283] PROGMEM = {
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

  log_end_request("/favicon.ico");
}

#if HISTORY_COUNT
void httpChartIconHandler(AsyncWebServerRequest *request)
{
  log_start_request(request, "/chart.png");

  static const uint8_t iconData[1426] PROGMEM = {
  0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D,
  0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20,
  0x08, 0x06, 0x00, 0x00, 0x00, 0x73, 0x7A, 0x7A, 0xF4, 0x00, 0x00, 0x00,
  0x07, 0x74, 0x49, 0x4D, 0x45, 0x07, 0xE4, 0x05, 0x18, 0x11, 0x24, 0x2A,
  0x2B, 0xFB, 0xE2, 0x03, 0x00, 0x00, 0x00, 0x09, 0x70, 0x48, 0x59, 0x73,
  0x00, 0x00, 0x1E, 0xC2, 0x00, 0x00, 0x1E, 0xC2, 0x01, 0x6E, 0xD0, 0x75,
  0x3E, 0x00, 0x00, 0x00, 0x04, 0x67, 0x41, 0x4D, 0x41, 0x00, 0x00, 0xB1,
  0x8F, 0x0B, 0xFC, 0x61, 0x05, 0x00, 0x00, 0x05, 0x21, 0x49, 0x44, 0x41,
  0x54, 0x78, 0xDA, 0xED, 0x57, 0x7D, 0x6C, 0x14, 0x45, 0x14, 0x9F, 0xFD,
  0xB8, 0xDB, 0x3B, 0x6C, 0xAF, 0x4D, 0x0B, 0xAD, 0x57, 0xA9, 0x96, 0x2F,
  0x63, 0x52, 0xA5, 0x15, 0xA9, 0x84, 0x68, 0x94, 0x22, 0x22, 0x0D, 0x41,
  0x04, 0x63, 0xAD, 0x06, 0x30, 0x31, 0x84, 0x46, 0x8D, 0x54, 0x4B, 0xB4,
  0x36, 0x92, 0x90, 0x8B, 0x1A, 0x91, 0xA8, 0x84, 0x88, 0xC8, 0x1F, 0xB6,
  0x48, 0xA5, 0x01, 0x5A, 0x51, 0x5B, 0x52, 0x90, 0x08, 0x69, 0x89, 0x69,
  0xAA, 0x06, 0x43, 0x55, 0x5A, 0x4C, 0x69, 0x9B, 0x52, 0x5A, 0x68, 0xE8,
  0xC7, 0x72, 0x7B, 0xDB, 0xBB, 0xDB, 0xDB, 0x9D, 0x19, 0xDF, 0xDC, 0xDD,
  0xF6, 0x9A, 0x72, 0xD7, 0x16, 0xD2, 0xE2, 0x3F, 0xBE, 0xDC, 0xEC, 0xCC,
  0xCE, 0xEC, 0x9B, 0xF7, 0x9B, 0xF7, 0x7E, 0xF3, 0x66, 0x0E, 0xA1, 0xFF,
  0xE5, 0x3F, 0x16, 0xCE, 0x6C, 0xE4, 0xE5, 0x2E, 0xDB, 0xCD, 0xF1, 0xFC,
  0x5A, 0x4C, 0x48, 0xCD, 0xA9, 0xFA, 0x86, 0xB7, 0xEF, 0x14, 0x00, 0xD1,
  0x6C, 0x0C, 0xC8, 0x43, 0xD9, 0x1C, 0xE2, 0xE6, 0x50, 0x44, 0xB3, 0xEF,
  0xA4, 0x07, 0x46, 0x00, 0x50, 0x42, 0xC0, 0x1F, 0x1C, 0xA2, 0x94, 0xDE,
  0x49, 0xFB, 0x11, 0x00, 0x88, 0xB2, 0x5F, 0xF0, 0x31, 0xA5, 0xF2, 0x73,
  0x7C, 0x7C, 0xB2, 0x96, 0x9C, 0x7C, 0x54, 0xD6, 0xB4, 0xFB, 0x64, 0xBF,
  0x7F, 0xEB, 0x56, 0x59, 0x3E, 0x39, 0x7A, 0x9C, 0x37, 0x1B, 0x84, 0x12,
  0xF0, 0x02, 0x0D, 0xD6, 0x53, 0x29, 0x7C, 0x52, 0xD2, 0x26, 0xBF, 0xCF,
  0xF7, 0x94, 0xCF, 0xEF, 0x9F, 0xAF, 0x63, 0xFC, 0x61, 0x6C, 0x0F, 0x80,
  0xF1, 0x20, 0x25, 0xA7, 0x38, 0x04, 0x5E, 0x4D, 0xBB, 0x62, 0xCE, 0x89,
  0x29, 0xED, 0x8C, 0x09, 0xC0, 0x8C, 0xFD, 0x54, 0x73, 0xA0, 0x45, 0x55,
  0x1F, 0x4F, 0x10, 0x84, 0x23, 0x3C, 0xC7, 0xFD, 0x6A, 0xF1, 0x78, 0x0E,
  0x8E, 0x03, 0x80, 0xB9, 0xFE, 0xD6, 0x48, 0xF8, 0x42, 0xD5, 0x85, 0x38,
  0x3B, 0x09, 0x6C, 0x73, 0x7B, 0x64, 0xEE, 0x52, 0xCF, 0xE0, 0xA7, 0xAD,
  0xAE, 0x7C, 0x75, 0xF4, 0xF8, 0xAE, 0xB8, 0xB8, 0x75, 0xB0, 0xEA, 0x9C,
  0x56, 0xB7, 0x7B, 0xF9, 0x17, 0x08, 0x69, 0xD1, 0xE6, 0x88, 0x84, 0x80,
  0x51, 0x90, 0x86, 0xEA, 0xC9, 0x4A, 0xA2, 0xD5, 0xB2, 0xBD, 0x5F, 0x56,
  0x4B, 0x6E, 0xA8, 0x1A, 0x4A, 0x98, 0x21, 0xD8, 0xA1, 0xAB, 0xC4, 0x1C,
  0xDB, 0x2D, 0x49, 0x19, 0x18, 0xA1, 0x4F, 0xB0, 0x61, 0xAC, 0x8A, 0x65,
  0x9C, 0x49, 0x84, 0x84, 0x84, 0x06, 0x57, 0xCF, 0xEA, 0xC9, 0x8A, 0x4F,
  0xC7, 0x89, 0x84, 0xE9, 0x40, 0xD1, 0x0D, 0x92, 0xFF, 0xD8, 0x67, 0xC7,
  0xD7, 0xCD, 0x7F, 0x73, 0x8F, 0xB4, 0x03, 0x21, 0x2B, 0x95, 0xA4, 0x6F,
  0xC1, 0xA7, 0xA5, 0xA5, 0x9A, 0xD6, 0x39, 0xDE, 0x1C, 0x23, 0x00, 0xD8,
  0xEA, 0x19, 0x00, 0x33, 0x02, 0x27, 0xD3, 0xD3, 0x33, 0x7F, 0xC8, 0xC8,
  0x38, 0x76, 0xD0, 0xE9, 0xFC, 0x7C, 0x0F, 0x42, 0x52, 0x34, 0x65, 0xD9,
  0x3D, 0x64, 0xC5, 0x84, 0x36, 0x43, 0xE4, 0x8E, 0x5B, 0x2D, 0xD6, 0xB7,
  0x80, 0xC3, 0x05, 0x29, 0xE9, 0x19, 0xE7, 0x2E, 0x6E, 0x2C, 0xF9, 0xDE,
  0x93, 0x98, 0xDA, 0xF6, 0xAE, 0xAA, 0x1E, 0x9B, 0x68, 0x11, 0x31, 0x39,
  0x20, 0xF0, 0xFC, 0x3E, 0x9F, 0xAA, 0x3E, 0xA1, 0x62, 0x8C, 0xB0, 0xC3,
  0xD1, 0x86, 0x14, 0x65, 0xFF, 0x68, 0xC5, 0xD5, 0x5F, 0x37, 0x16, 0x10,
  0x62, 0xCC, 0xF5, 0xF6, 0xEA, 0x39, 0xBF, 0xB8, 0x56, 0x1B, 0xE1, 0xEE,
  0x9A, 0xD2, 0x9C, 0xDC, 0xD7, 0xBA, 0xE6, 0x65, 0x15, 0xD7, 0x6E, 0xF9,
  0x68, 0x41, 0x36, 0xC1, 0xFB, 0xFD, 0x58, 0xDF, 0xFB, 0x4F, 0xAB, 0xED,
  0xE2, 0xDA, 0xBC, 0x7B, 0x96, 0x28, 0x37, 0x14, 0x77, 0x7D, 0x71, 0x5E,
  0x4B, 0x54, 0x0F, 0x84, 0x38, 0x40, 0x47, 0x38, 0xA0, 0x6B, 0x9A, 0x1E,
  0x0C, 0x0D, 0xF4, 0x25, 0x89, 0x62, 0x61, 0x5D, 0x4A, 0xCA, 0x73, 0xF5,
  0x61, 0xC0, 0xAB, 0xBF, 0x6A, 0x5A, 0xC0, 0x21, 0xFE, 0x03, 0x70, 0xFE,
  0x86, 0xB3, 0xAE, 0x5C, 0xD3, 0x38, 0xFA, 0xD2, 0x66, 0x4B, 0x9F, 0x7D,
  0xE9, 0xFC, 0xB6, 0xC5, 0x27, 0xCA, 0xF3, 0x74, 0xC9, 0xBF, 0x88, 0x60,
  0xFA, 0xBB, 0x05, 0x09, 0xDF, 0xAC, 0x59, 0x79, 0x77, 0xB3, 0x24, 0x8A,
  0x8D, 0x16, 0x2B, 0xD7, 0xBC, 0x74, 0xE7, 0x77, 0xCF, 0x44, 0xF5, 0x00,
  0x7C, 0x0C, 0x99, 0x38, 0x12, 0x02, 0x0D, 0xE3, 0x42, 0x00, 0x54, 0x8A,
  0x30, 0xEE, 0x49, 0x96, 0xA4, 0x06, 0x9E, 0xE7, 0x5F, 0x57, 0x53, 0x53,
  0x3F, 0x3E, 0x6C, 0x77, 0x94, 0x1D, 0xB1, 0x71, 0x2F, 0x1A, 0x86, 0x51,
  0xF4, 0xD3, 0x96, 0xDC, 0x1E, 0x53, 0x1F, 0xE2, 0x2E, 0x0A, 0x76, 0x7B,
  0x25, 0xC6, 0xF8, 0xFD, 0x6D, 0x8A, 0xD2, 0x8E, 0xDE, 0xD9, 0xC4, 0xBA,
  0xCB, 0xA1, 0x1C, 0x58, 0x78, 0xF4, 0x42, 0x93, 0xDB, 0xE3, 0x46, 0xC3,
  0xFE, 0x80, 0xE8, 0x0B, 0x68, 0xCB, 0xA1, 0xEF, 0xD4, 0x4D, 0x00, 0x50,
  0x38, 0x04, 0x26, 0x82, 0xF5, 0xFD, 0xFD, 0x1D, 0x50, 0x6D, 0x0E, 0xBE,
  0x78, 0x3C, 0xEC, 0xD9, 0x50, 0xE5, 0x74, 0xDE, 0xDB, 0x94, 0x5F, 0x74,
  0xEA, 0x81, 0xC6, 0xBA, 0xB4, 0x87, 0x6B, 0xF6, 0xAF, 0x29, 0x48, 0x4A,
  0xEA, 0x92, 0x28, 0xED, 0x15, 0x13, 0x13, 0x9F, 0x1F, 0x72, 0xBB, 0x97,
  0x1A, 0x84, 0xB4, 0xBE, 0xA1, 0x28, 0x47, 0xC7, 0x84, 0x99, 0x06, 0x30,
  0xD9, 0x0B, 0xB3, 0x2E, 0x82, 0xA2, 0x06, 0x30, 0x3D, 0x1C, 0x9D, 0x84,
  0xE1, 0x10, 0xD0, 0x71, 0xB6, 0xE1, 0xE1, 0x5D, 0x75, 0x8F, 0xF6, 0x64,
  0x2E, 0x1D, 0x4A, 0xBA, 0xDE, 0x3B, 0x9B, 0xA7, 0xF4, 0x1C, 0x24, 0x97,
  0x43, 0x71, 0x0E, 0x47, 0x87, 0xDF, 0xEB, 0x2D, 0xC3, 0x82, 0xB0, 0xD9,
  0x87, 0xF1, 0x81, 0x68, 0x7A, 0xD5, 0x2F, 0x2F, 0x3C, 0xD4, 0x37, 0xE0,
  0x9D, 0x65, 0xF1, 0x0C, 0xA6, 0xB5, 0xBA, 0x36, 0x34, 0x47, 0x27, 0x61,
  0x38, 0x15, 0xC7, 0x4A, 0x44, 0xEB, 0x2B, 0xFF, 0x9A, 0x2B, 0x50, 0xB2,
  0x13, 0x21, 0x63, 0x79, 0x49, 0x6D, 0x39, 0x73, 0x49, 0x19, 0x73, 0x71,
  0x9D, 0xD5, 0x3A, 0x48, 0x80, 0xA8, 0x7E, 0x5D, 0x47, 0x01, 0x42, 0x32,
  0xA0, 0xEF, 0xB7, 0x68, 0xFA, 0x7F, 0xBC, 0xF7, 0xB4, 0x3B, 0x5A, 0x7F,
  0x84, 0x84, 0xE1, 0x6D, 0x18, 0xCD, 0x01, 0xAB, 0xF6, 0x9C, 0x90, 0xAC,
  0x3C, 0x57, 0x89, 0x78, 0xAE, 0xB8, 0xFA, 0x95, 0xC5, 0xDD, 0xA3, 0xDD,
  0x0B, 0x89, 0x66, 0x3B, 0x24, 0x8F, 0xEB, 0xA0, 0x7C, 0xC6, 0x3B, 0x3C,
  0x5C, 0x87, 0x6E, 0x51, 0x22, 0x24, 0xA4, 0x18, 0x1C, 0x30, 0x26, 0x15,
  0xEF, 0xA0, 0xFC, 0x8A, 0xB4, 0xD3, 0xB3, 0x13, 0x13, 0x9C, 0xC5, 0x10,
  0x9A, 0xA6, 0xEA, 0x97, 0xB2, 0x6A, 0xC7, 0x4E, 0xF0, 0x6C, 0x7F, 0xFF,
  0x3E, 0xC8, 0x13, 0x65, 0x43, 0xB0, 0x71, 0x5C, 0x30, 0xCD, 0x6D, 0x03,
  0x30, 0xEF, 0x02, 0x11, 0x0E, 0x50, 0xEE, 0xD5, 0xAC, 0xD6, 0x1F, 0xDD,
  0xCA, 0x8C, 0x95, 0x6E, 0xB9, 0x2F, 0x60, 0xA8, 0xD8, 0x19, 0x6B, 0x92,
  0xA2, 0x71, 0x52, 0xED, 0x44, 0x12, 0x21, 0x21, 0x09, 0x67, 0xC2, 0xF0,
  0x1A, 0x36, 0x56, 0x75, 0xA6, 0xFB, 0x03, 0x78, 0x8D, 0xC7, 0xA7, 0x49,
  0xB2, 0xA2, 0xC4, 0x5F, 0x1E, 0xBA, 0x36, 0xEF, 0x76, 0x8D, 0x4C, 0x0A,
  0x00, 0xB3, 0x1C, 0xE2, 0x40, 0x08, 0x41, 0xF7, 0xAC, 0xEE, 0xAB, 0x3A,
  0x26, 0x7F, 0xB2, 0x36, 0x5C, 0x54, 0xDB, 0x3D, 0x56, 0xAD, 0x7D, 0x5A,
  0x01, 0x84, 0x6E, 0x44, 0x64, 0xE4, 0x46, 0x74, 0x36, 0x37, 0xD7, 0x90,
  0x55, 0xE3, 0xC9, 0x00, 0x46, 0x2B, 0x04, 0x2B, 0x59, 0x72, 0xCD, 0x55,
  0xE8, 0x9D, 0x0E, 0x00, 0xA3, 0xB6, 0x21, 0x3C, 0x20, 0x13, 0x8A, 0x02,
  0x3F, 0x13, 0x5A, 0x16, 0x28, 0xFA, 0xE9, 0xC2, 0xC5, 0x6C, 0xEB, 0x9C,
  0x99, 0x0E, 0xC3, 0x37, 0x01, 0x30, 0x85, 0xE3, 0xF8, 0x07, 0x1F, 0x59,
  0x30, 0xA7, 0x0D, 0xA2, 0x71, 0xDB, 0xC4, 0x9A, 0x48, 0xE0, 0xE0, 0xEB,
  0x3B, 0xDF, 0x71, 0x79, 0xD9, 0x18, 0x00, 0x2C, 0x13, 0xF1, 0x70, 0xAE,
  0x1B, 0x48, 0x56, 0x3C, 0x19, 0xD3, 0xB9, 0x6A, 0x90, 0xFB, 0x6F, 0xF2,
  0x80, 0x68, 0xB1, 0xEE, 0x15, 0x45, 0xE1, 0x02, 0x0F, 0x7F, 0x8F, 0x38,
  0xC8, 0xB1, 0x3C, 0x2F, 0xC0, 0x89, 0x2C, 0x04, 0x39, 0x22, 0xD9, 0x25,
  0x78, 0x11, 0xC5, 0xC8, 0xFF, 0xA8, 0x89, 0x96, 0xC8, 0x88, 0x6B, 0x18,
  0x9A, 0x4F, 0xC3, 0xC1, 0xA5, 0x11, 0xA0, 0x31, 0xA4, 0x4B, 0xC8, 0xB6,
  0xF0, 0x0B, 0xBE, 0x5D, 0x1D, 0xF1, 0x78, 0x45, 0x45, 0xC5, 0x5D, 0x1C,
  0xC7, 0x3D, 0x04, 0x3B, 0x20, 0x1D, 0xEA, 0x59, 0xD0, 0x37, 0x13, 0x6A,
  0x3B, 0x68, 0x38, 0xA0, 0xCD, 0xC6, 0x38, 0x18, 0xB3, 0x41, 0xC5, 0xDE,
  0x27, 0x0D, 0x01, 0x44, 0x01, 0x1D, 0x3F, 0x0D, 0x65, 0xB6, 0x61, 0x38,
  0x4D, 0x15, 0x68, 0xFA, 0xA0, 0x3D, 0x00, 0x75, 0x3F, 0x8C, 0x5D, 0x81,
  0xFA, 0x6F, 0x11, 0x8E, 0xCF, 0x78, 0x51, 0x14, 0x73, 0x60, 0x20, 0x13,
  0x4A, 0x1A, 0x14, 0x07, 0x0C, 0x24, 0xC0, 0x07, 0xCC, 0x3B, 0x36, 0xE6,
  0x00, 0x50, 0x66, 0x20, 0x4C, 0xE3, 0xEC, 0x76, 0x24, 0xC4, 0x30, 0xCC,
  0x56, 0x1C, 0xE4, 0x0E, 0xE8, 0x30, 0xDB, 0x14, 0xE6, 0x61, 0xEF, 0x0C,
  0x08, 0xBB, 0x37, 0x30, 0x52, 0x2B, 0x50, 0x98, 0x07, 0x5A, 0xC0, 0x76,
  0xD7, 0xBF, 0x0D, 0xDC, 0xA6, 0x9D, 0xF0, 0x07, 0x7E, 0xD0, 0x00, 0x00,
  0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82
};
  AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", iconData, sizeof(iconData));
  request->send(response);

  log_end_request("/chart.png");
}
#endif  // HISTORY_COUNT
