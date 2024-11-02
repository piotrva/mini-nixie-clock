#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

#include "credentials.h"

// Connections:
// ESP32   <->   Nixie
// GND           1 GND
//               2 SWDIO
//               3 SWCLK
//               4 NRST
// 3V3           5 3.3V
// 20 (RX)       6 TX USART2
// 21 (TX)       7 RX USART2

// Edit below to change how often to check time synchronization between Nixie and NTP. 15s default
#define TIME_CHECK_INTERVAL_MS (15UL*1000UL)
// Edit below to change what time difference shall trigger updating time on nixie clock. 30s default
#define TIME_NIXIE_UPDATE_DIFFERENCE_S (30UL)


// Do not edit - internal timeout for serial communication with Nixie clock
#define TIME_GET_NIXIE_TIMEOUT_MS (1000UL)

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void setTimezone(String timezone){
  Serial.printf("  Setting Timezone to %s\n",timezone.c_str());
  Serial.println("");
  setenv("TZ",timezone.c_str(),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

void initTime(String timezone){
  struct tm timeinfo;

  Serial.println("Setting up time");
  configTime(0, 0, "pool.ntp.org");    // First connect to NTP server, with 0 TZ offset
  if(!getLocalTime(&timeinfo)){
    Serial.println("  Failed to obtain time");
    return;
  }
  Serial.println("  Got the time from NTP");
  // Now we can set the real timezone
  setTimezone(timezone);
}

bool getNixieTime(struct tm* timeToGet)
{
  // Flush RX buffer
  while (Serial1.available())
  {
    Serial1.read();
  }
  // Send get time command to nixie
  Serial1.println("GT");
  String response = Serial1.readStringUntil('\n');
  if (response.isEmpty() || response.length() != 9)
  {
    timeToGet->tm_hour = 0;
    timeToGet->tm_min  = 0;
    timeToGet->tm_sec  = 0;
    // Serial.println("Nixie query timeout");
    return false;
  }
  else
  {
    // Serial.print("Nixie query response: ");
    sscanf(response.c_str(), "%i %i %i", &(timeToGet->tm_hour), &(timeToGet->tm_min), &(timeToGet->tm_sec));
    // Serial.printf("%02d %02d %02d\r\n", timeToGet->tm_hour, timeToGet->tm_min, timeToGet->tm_sec);
    return true;
  }
}

void timePrintSerial(struct tm* timeToPrint)
{
  Serial.printf("%02d %02d %02d\r\n", timeToPrint->tm_hour, timeToPrint->tm_min, timeToPrint->tm_sec);
}

unsigned long tmDiffHour(struct tm* time1, struct tm* time2)
{
  unsigned long time1s, time2s;
  time1s = time1->tm_hour * 3600 + time1->tm_min * 60 + time1->tm_sec;
  time2s = time2->tm_hour * 3600 + time2->tm_min * 60 + time2->tm_sec;
  if (time1s > time2s)
  {
    return time1s - time2s;
  }
  else
  {
    return time2s - time1s;
  }
}

struct tm nixieTime;
struct tm localTime;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  Serial1.setTimeout(TIME_GET_NIXIE_TIMEOUT_MS);
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(WIFI_SSDI, WIFI_PASS);
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSDI);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("");
  Serial.println("WiFi connected.");

  initTime("CET-1CEST,M3.5.0,M10.5.0/3");   // Set for Europe/Warsaw
  digitalWrite(LED_BUILTIN, LOW);
}

unsigned long lastTimeCheck = 0;

void loop() {
  if (millis() - lastTimeCheck > TIME_CHECK_INTERVAL_MS)
  {
    lastTimeCheck = millis();
    if (getLocalTime(&localTime) && getNixieTime(&nixieTime))
    {
      Serial.print("Nixie: ");
      timePrintSerial(&nixieTime);
      Serial.print("NTP  : ");
      timePrintSerial(&localTime);
      unsigned long tdiff = tmDiffHour(&nixieTime, &localTime);
      Serial.print("Difference: ");
      Serial.println(tdiff);
      if (tdiff > TIME_NIXIE_UPDATE_DIFFERENCE_S)
      {
        Serial.println("Update Nixie now!");
        Serial1.printf("ST %02d %02d %02d\r\n", localTime.tm_hour, localTime.tm_min, localTime.tm_sec);
      }
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
      Serial.println("Failed to obtain NTP or Nixie time...");
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}
