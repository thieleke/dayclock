#if HISTORY_COUNT
history_t *get_history()
{
  return get_history(historyPos);
}

history_t *get_history(int pos)
{
  if (pos < 0 || pos >= HISTORY_COUNT)
    pos = 0;

  return &historyVals[pos];
}

history_t *get_current_history()
{
  return get_history(historyPos);
}

history_t *get_previous_history()
{
  return get_previous_history(historyPos);
}

history_t *get_previous_history(int pos)
{
  pos = pos - 1 < 0 ? (HISTORY_COUNT - 1) : pos - 1;
  return get_history(pos);
}

int get_next_index()
{
  return get_next_index(historyPos);
}

int get_next_index(int pos)
{
  return pos + 1 > (HISTORY_COUNT - 1) ? 0 : pos + 1;
}

history_t *get_next_history()
{
  return get_next_history(historyPos);
}

history_t *get_next_history(int pos)
{
  pos = get_next_index(pos);
  return get_history(pos);
}

int add_history(time_t timestamp, float temp_c, float humidity, unsigned int co2)
{
  // Don't update history if it is being accessed
  if (isHistoryBusy())
    return historyPos;

  in_add_history = true;  
  //Serial.printf("Adding History[%d]: %lu, %0.2f, %0.2f, %u\n", historyPos, timestamp, temp_c, humidity, co2);

  history_t *h;
  h = get_next_history();
  historyPos = get_next_index();

  h->timestamp = timestamp;
  h->temperature = temp_c;
  h->humidity = humidity;
  h->co2 = co2;

  in_add_history = false;
  return historyPos;
}
#endif // HISTORY_COUNT


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
  return ((c * 1.8) + 32);
}

float round_to_tenths(float val)
{
  double d = round((double)val * 10.0) / 10.0;
  return float(d);
}

char* localTime()
{
  memset(localTimeBuffer, 0, sizeof(localTimeBuffer));
  struct tm t;
  if (!getLocalTime(&t)) {
    //Serial.println("Failed to obtain time");
    updateNTP();
    return localTimeBuffer;
  }

  strftime(localTimeBuffer, sizeof(localTimeBuffer), "%c", &t);
  return localTimeBuffer;
}

bool wifi_connect()
{
  Serial.printf("wifi_connect() - status = %d [%lu]\n", WiFi.status(), millis());
  if (WiFi.status() == WL_CONNECTED)
  {
    wifiConnectedTicks = millis();    
    digitalWrite(LED_BUILTIN, HIGH);
    return true;
  }

  Serial.printf("Connecting to Wi-Fi (%s)\n", ssid);
  lcd_print("Connecting to", 0);
  lcd_print(ssid, 1);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // https://github.com/me-no-dev/ESPAsyncWebServer/pull/743
  WiFi.begin(ssid, password);
  for (int i = 0; i < 10; i++)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.print(".");
      delay(500);
      yield();
      continue;
    }

    break;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.printf("\nwifi_connect() - connected to %s (%s) [%lu]\n", ssid, WiFi.localIP().toString().c_str(), millis());
    lcd_print("Wi-Fi connected", 0);
    lcd_print(WiFi.localIP().toString(), 1);
    wifiConnectedTicks = millis();
    return true;
  }
  else
  {
    Serial.printf("Wi-Fi connection failed [%lu]\n", millis());
    lcd_print("Wi-Fi connection failed", 0);
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
}

void lcd_print(String msg, int line)
{
  if (line < 0 || line > LCD_ROWS - 1)
    return;

  while (msg.length() < LCD_COLUMNS)
    msg += " ";

  lcd.setCursor(0, line);
  lcd.print(msg.c_str());
}

void log_free_memory()
{
  log_free_memory("MEMORY");
}

void log_free_memory(const char *msg)
{
  Serial.printf("%s: Free Heap = %d, Max Available Heap = %d\n", msg, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
}
