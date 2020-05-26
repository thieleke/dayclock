history_t *get_history()
{
  Serial.printf("get_history() - historyPos = %d\n", historyPos);
  return get_history(historyPos);
}

history_t *get_history(int pos)
{
  if(pos < 0 || pos >= HISTORY_COUNT)
    pos = 0;

  return &history[pos];
}

history_t *get_current_history()
{
  Serial.println("get_current_history()");
  return get_history(historyPos);
}

history_t *get_previous_history()
{
  Serial.println("get_previous_history()");
  return get_previous_history(historyPos);
}

history_t *get_previous_history(int pos)
{
  Serial.printf("get_previous_history(%d)\n", pos);
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
  Serial.println("get_next_history()");
  return get_next_history(historyPos);
}

history_t *get_next_history(int pos)
{
  Serial.printf("get_next_history(%d)\n", pos);
  pos = get_next_index(pos);
  return get_history(pos);  
}

int add_history(unsigned int timestamp, float temp_c, float humidity, unsigned int co2)
{  
  Serial.printf("!!! Adding history: historyPos = %d\n", historyPos);
  history_t *h;
  h = get_next_history();
    
  h->timestamp = timestamp;
  h->temperature = temp_c;
  h->humidity = humidity;
  h->co2 = co2;
  Serial.printf("historyPos = %d\n", historyPos);
  Serial.printf("* add_history(): %lu\n", h->timestamp);
  historyPos = get_next_index();
  Serial.printf("historyPos is now %d !!!!!\n", historyPos);
  return historyPos;
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
  return((c * 1.8) + 32);
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

void(* resetFunc)(void) = 0;
