// MIT License

// Copyright (c) 2026 Grzegorz Rompa, Tomasz Łuczak

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <SPI.h>
#include <Wire.h>
#include <GxEPD2_BW.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sys/time.h>
#include <SensirionI2cSht4x.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

#include "stacja.h"
#include "secrets.h"
#include "locations.h"


// ===== Pin mapping dla reTerminal E1001 =====
#define EPD_SCK_PIN   7
#define EPD_MOSI_PIN  9
#define EPD_CS_PIN    10
#define EPD_DC_PIN    11
#define EPD_RES_PIN   12
#define EPD_BUSY_PIN  13

// Magistrala I2C dla wbudowanego czujnika SHT4x
#define SHT4X_SDA_PIN 19
#define SHT4X_SCL_PIN 20
#define RTC_I2C_ADDR 0x51

// ===== Peryferia =====
const int BUZZER_PIN         = 45;   // Buzzer na GPIO45
const int BUTTON_BUZZ_PIN    = 3;    // Przycisk buzzera na GPIO3
const int BUTTON_SCREEN_PIN  = 4;    // Przycisk zmiany ekranów na GPIO4
const int BATTERY_ADC_PIN    = 1;    // GPIO1 - Battery voltage ADC
const int BATTERY_ENABLE_PIN = 21; // GPIO21 - Battery monitoring enable

SPIClass hspi(HSPI);
SensirionI2cSht4x sht4x;

// ===== Display: 7.5" B&W 800x480 =====
#define MAX_DISPLAY_BUFFER_SIZE 16000u
#define MAX_HEIGHT(EPD) \
    (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) \
         ? EPD::HEIGHT \
         : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))

GxEPD2_BW<GxEPD2_750_GDEY075T7, MAX_HEIGHT(GxEPD2_750_GDEY075T7)>
    display(GxEPD2_750_GDEY075T7(EPD_CS_PIN, EPD_DC_PIN, EPD_RES_PIN, EPD_BUSY_PIN));

// Zmienne czasu i stanu
int hours = 12;
int minutes = 0;
char lastSyncTimeStr[16] = "--:--";
char currentDateStr[16]  = "--.--.----";

// System ekranów (0: Główny, 1: Home 7 dni, 2: Stogi 7 dni)
int currentScreen = 0;
bool lastScreenBtnState = HIGH;
unsigned long lastScreenDebounce = 0;

// Zmienne dla trybu ciągłego i obsługi przycisku buzzera
bool buzzerActive = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned long lastBuzzerMillis = 0;
unsigned long lastWeatherMillis = 0;
unsigned long lastClockMillis = 0;
unsigned long lastFullRefreshMillis = 0;
unsigned long lastBatteryMillis = 0;

int batteryPercentage = 100;
float batteryVoltage = 0.0;
float batteryMinV = 99.0;
float batteryMaxV = 0.0;

struct WeatherData {
  float temperature = 0.0;
  float precipitation = 0.0;
  float windSpeed = 0.0;
  float windGusts = 0.0;
  int windDirectionDeg = 0;
  bool valid = false;
};

struct HourlyData {
  float temp[12];
  float precip[12];
  float windSpeed[12];
  int hour[12];
  bool valid = false;
};

struct DailyForecast {
  char date[16][12];
  float tMax[7];
  float tMin[7];
  float precipitation[7];
  float windSpeedMax[7];
  int windDirectionMaxDeg[7];
  bool valid = false;
};

WeatherData meteoHome;
WeatherData meteoSecond;
HourlyData hourlyHome;
DailyForecast forecastHome;
DailyForecast forecastSecond;

// Zmienne dla czujnika wewnętrznego SHT4x
float internalTemp = 0.0;
int internalHumidity = 0;
bool sht4xValid = false;
uint16_t sht4xLastError = 0;
bool wifiLastSuccess = false;
bool sensorReadLastSuccess = false;
unsigned long lastNtpSyncMillis = 0;
bool ntpEverSynced = false;

bool connectWiFi();
void disconnectWiFi();
bool syncTimeNTP();
bool syncTimeFromRtc();
bool updateRTCFromSystemTime();
void updateWeatherFromAPI();
bool initSht4x();
void readInternalSensor();
void readBatteryLevel();
const char* getWindDirectionText(int degrees);
const char* getDayOfWeekShort(const char* dateStr);
void drawDashboard();
void drawForecastScreen(const char* title, DailyForecast &forecast);
void updateBuzzerAndBatteryStatusOnScreen();
void updateClockAndDateOnScreen();

uint8_t bcdToDec(uint8_t val) {
  return ((val >> 4) * 10) + (val & 0x0F);
}

uint8_t decToBcd(uint8_t val) {
  return ((val / 10) << 4) | (val % 10);
}

void setup()
{
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 5000)) {
    delay(10);
  }
  Serial.println(F("[E1001] Start: Optimized power mode active"));
  delay(100);

  // Obniżenie taktowania procesora do 80 MHz dla oszczędności energii
  setCpuFrequencyMhz(80);

  Wire.begin(SHT4X_SDA_PIN, SHT4X_SCL_PIN);
  Wire.setClock(100000);

  // Sync time from RTC before Wi-Fi
  syncTimeFromRtc();

  initSht4x();
  readInternalSensor();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_BUZZ_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SCREEN_PIN, INPUT_PULLUP);
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BATTERY_ENABLE_PIN, OUTPUT);
  digitalWrite(BATTERY_ENABLE_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  pinMode(EPD_RES_PIN, OUTPUT);
  pinMode(EPD_DC_PIN,  OUTPUT);
  pinMode(EPD_CS_PIN,  OUTPUT);

  hspi.begin(EPD_SCK_PIN, -1, EPD_MOSI_PIN, -1);
  display.epd2.selectSPI(hspi, SPISettings(2000000, MSBFIRST, SPI_MODE0));
  display.init(0);

  Serial.end();
esp_log_level_set("*", ESP_LOG_NONE);

  delay(100);

  readBatteryLevel();

  // Pobranie danych przez Wi-Fi i natychmiastowe rozłączenie radia
  if (connectWiFi()) {
    wifiLastSuccess = true;
    if (syncTimeNTP()) {
      ntpEverSynced = true;
      lastNtpSyncMillis = millis();
      updateRTCFromSystemTime();
    }
    updateWeatherFromAPI();
    disconnectWiFi();
  } else {
    wifiLastSuccess = false;
    disconnectWiFi();
  }

  drawDashboard();

  unsigned long currentMillis = millis();
  lastClockMillis = currentMillis;
  lastWeatherMillis = currentMillis;
  lastFullRefreshMillis = currentMillis;
  lastBuzzerMillis = currentMillis;
  lastBatteryMillis = currentMillis;
}

void loop()
{
  unsigned long currentMillis = millis();

  int screenReading = digitalRead(BUTTON_SCREEN_PIN);
  if (screenReading != lastScreenBtnState) {
    lastScreenDebounce = currentMillis;
  }
  if ((currentMillis - lastScreenDebounce) > debounceDelay) {
    static int currentScreenState = HIGH;
    if (screenReading != currentScreenState) {
      currentScreenState = screenReading;
      if (screenReading == LOW) {
        readInternalSensor();
        currentScreen = (currentScreen + 1) % 3;
        drawDashboard();
      }
    }
  }
  lastScreenBtnState = screenReading;

  int reading = digitalRead(BUTTON_BUZZ_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = currentMillis;
  }
  if ((currentMillis - lastDebounceTime) > debounceDelay) {
    static int currentState = HIGH;
    if (reading != currentState) {
      currentState = reading;
      if (currentState == LOW) {
        buzzerActive = !buzzerActive;
        if (buzzerActive) {
          lastBuzzerMillis = currentMillis;
        } else {
          noTone(BUZZER_PIN);
          digitalWrite(BUZZER_PIN, LOW);
        }
        if (currentScreen == 0) {
          updateBuzzerAndBatteryStatusOnScreen();
        }
      }
    }
  }
  lastButtonState = reading;

  if (buzzerActive && (currentMillis - lastBuzzerMillis >= buzzerPeriod)) {
    lastBuzzerMillis = currentMillis;
    tone(BUZZER_PIN, 2000, 150);
  }

  if (currentMillis - lastClockMillis >= 60000) {
    lastClockMillis = currentMillis;
    minutes++;
    if (minutes >= 60) {
      minutes = 0;
      hours = (hours + 1) % 24;
    }
    snprintf(lastSyncTimeStr, sizeof(lastSyncTimeStr), "%02d:%02d", hours, minutes);
    if (currentScreen == 0) {
      updateClockAndDateOnScreen();
    }
  }

  if (currentMillis - lastBatteryMillis >= batteryRefreshPeriod) {
    lastBatteryMillis = currentMillis;
    readBatteryLevel();
    updateBuzzerAndBatteryStatusOnScreen();
  }

  // Pełne odświeżanie ekranu co 1 godzinę (dla oszczędności energii matrycy)
  if (currentMillis - lastFullRefreshMillis >= fullRefreshPeriod) {
    lastFullRefreshMillis = currentMillis;
    readInternalSensor(); 
    drawDashboard();
  }

  // Aktualizacja pogody i czasu co 30 minut (włączanie Wi-Fi -> NTP + pogoda -> rozłączenie)
  if (currentMillis - lastWeatherMillis >= weatherRefreshPeriod) {
    lastWeatherMillis = currentMillis;
    readInternalSensor();
    if (connectWiFi()) {
      wifiLastSuccess = true;
      if (!ntpEverSynced || (currentMillis - lastNtpSyncMillis >= ntpRefreshperiod)) {
        if (syncTimeNTP()) {
          ntpEverSynced = true;
          lastNtpSyncMillis = currentMillis;
          updateRTCFromSystemTime();
        }
      }
      updateWeatherFromAPI();
      disconnectWiFi();
    } else {
      wifiLastSuccess = false;
      disconnectWiFi();
    }
      drawDashboard();
  }
  esp_sleep_enable_timer_wakeup(100000); // 100 ms
  esp_light_sleep_start();
}

bool initSht4x() {
  uint8_t sht4xAddress = 0x44;
  const uint8_t addrs[2] = {0x44, 0x45};

  for (int i = 0; i < 2; i++) {
    sht4xAddress = addrs[i];
    sht4x.begin(Wire, sht4xAddress);

    float temperature = 0.0f, humidity = 0.0f;
    uint16_t error = sht4x.measureHighPrecision(temperature, humidity);
    if (error == 0) {
      sht4xValid = true;
      internalTemp = temperature;
      internalHumidity = (int)(humidity + 0.5f);
      Serial.printf("[SHT4x] init OK addr=0x%02X\n", sht4xAddress);
      return true;
    }
    Serial.printf("[SHT4x] init FAIL addr=0x%02X error=%u\n", sht4xAddress, error);
  }
  sht4xValid = false;
  return false;
}

void readInternalSensor() {
  float temperature = 0.0f;
  float humidity = 0.0f;
  uint16_t error = 0xFFFF;

  // 3 próby
  for (int i = 0; i < 3; i++) {
    error = sht4x.measureHighPrecision(temperature, humidity);
    if (error == 0) {
      internalTemp = temperature;
      internalHumidity = (int)(humidity + 0.5);
      sht4xValid = true;
      sensorReadLastSuccess = true;
      return;
    }
    delay(20);
  }

  // Reinit after error
  sht4xLastError = error;

  Wire.end();
  delay(5);
  Wire.begin(SHT4X_SDA_PIN, SHT4X_SCL_PIN);
  Wire.setClock(100000);
  if (initSht4x()) {
    // initSht4x set temperature and humindity, ans valid=true
    sensorReadLastSuccess = true;
    return;
  }
  sht4xValid = false;
  sensorReadLastSuccess = false;
  // Serial.printf("[SHT4x] read failed err=%u\n", error);
}

void readBatteryLevel() {
  digitalWrite(BATTERY_ENABLE_PIN, HIGH);
  delay(5);
  int mv = analogReadMilliVolts(BATTERY_ADC_PIN);
  digitalWrite(BATTERY_ENABLE_PIN, LOW);

  batteryVoltage = (mv / 1000.0) * 2.0;

  if (batteryVoltage > batteryMaxV) batteryMaxV = batteryVoltage;
  if (batteryVoltage < batteryMinV) batteryMinV = batteryVoltage;

  // Kalibracja na podstawie testu: Max = 4.11V, Min = 2.58V
  float vMax = 4.11;
  float vMin = 2.58;
  
  int percent = (int)((batteryVoltage - vMin) / (vMax - vMin) * 100.0);
  if (percent > 100) percent = 100;
  if (percent < 0)   percent = 0;
  batteryPercentage = percent;
}

bool connectWiFi() {
  WiFi.persistent(false);


  for (int connectAttempt = 0; connectAttempt < 3; connectAttempt++) {
    WiFi.mode(WIFI_OFF);
    delay(120);

    WiFi.mode(WIFI_STA);
    delay(120);

    WiFi.disconnect(false, false);
    delay(80);


    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.setSleep(true);
      esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
      return true;
    }

    WiFi.disconnect(true, false);
    delay(120);
  }

  WiFi.mode(WIFI_OFF);
  return false;
}

void disconnectWiFi() {
  WiFi.disconnect(true, false);
  delay(80);
  WiFi.mode(WIFI_OFF);
}

void updateClockAndDateOnScreen() {
  if (currentScreen != 0) return;
  display.setPartialWindow(35, 55, 215, 70);
  display.firstPage();
  do {
    display.fillRect(35, 55, 215, 70, GxEPD_WHITE);
    
    char timeStr[8];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d", hours, minutes);
    int16_t tbx, tby; uint16_t tbw, tbh;
    
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeSansBold24pt7b);
    display.getTextBounds(timeStr, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(25 + (235 - tbw) / 2 - tbx, 95);
    display.print(timeStr);

    display.setFont(&FreeSansBold12pt7b);
    display.getTextBounds(currentDateStr, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(25 + (235 - tbw) / 2 - tbx, 120);
    display.print(currentDateStr);
  } while (display.nextPage());
}

void updateBuzzerAndBatteryStatusOnScreen() {
  if (currentScreen != 0) return;
  display.setPartialWindow(35, 235, 215, 95);
  display.firstPage();
  do {
    display.fillRect(35, 235, 215, 95, GxEPD_WHITE);
    
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeSans9pt7b);
    display.setCursor(35, 270);
    display.print("Buzzer 15min: ");
    display.print(buzzerActive ? "Aktywny" : "Wylaczony");

    display.setFont(&FreeSans9pt7b);
    display.setCursor(35, 298);
    display.print("Bateria:");
    char batBuf[32];
    snprintf(batBuf, sizeof(batBuf), "%d%% (%.2fV)", batteryPercentage, batteryVoltage);
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(35, 323);
    display.print(batBuf);
  } while (display.nextPage());
}

bool syncTimeNTP() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 15) {
    delay(500);
    retry++;
  }
  if (retry < 15 && timeinfo.tm_year > (2023 - 1900)) {
    hours = timeinfo.tm_hour;
    minutes = timeinfo.tm_min;
    snprintf(lastSyncTimeStr, sizeof(lastSyncTimeStr), "%02d:%02d", hours, minutes);
    snprintf(currentDateStr, sizeof(currentDateStr), "%02d.%02d.%04d", timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    return true;
  }
  return false;
}

bool syncTimeFromRtc() {
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write((uint8_t)0x02); // sekundy do rejestru
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)RTC_I2C_ADDR, 7) != 7) {
    return false;
  }

  uint8_t secReg = Wire.read();
  uint8_t minReg = Wire.read();
  uint8_t hourReg = Wire.read();
  uint8_t dayReg = Wire.read();
  Wire.read(); //day of week - noy used
  uint8_t monthReg = Wire.read();
  uint8_t yearReg = Wire.read();

  if (secReg & 0x80) {
    // VL=1: RTC has not accuracy time
    return false;
  }

  int sec = bcdToDec(secReg & 0x7F);
  int min = bcdToDec(minReg & 0x7F);
  int hour = bcdToDec(hourReg & 0x3F);
  int day = bcdToDec(dayReg & 0x3F);
  int month = bcdToDec(monthReg & 0x1F);
  int year = bcdToDec(yearReg) + ((monthReg & 0x80) ? 1900 : 2000);

  if (sec > 59 || min > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) {
    return false;
  }

  struct tm t = {0};
  t.tm_sec = sec;
  t.tm_min = min;
  t.tm_hour = hour;
  t.tm_mday = day;
  t.tm_mon = month - 1;
  t.tm_year = year - 1900;
  t.tm_isdst = -1;

  time_t epoch = mktime(&t);
  if (epoch <= 0 ) {
    return false;
  }

  struct timeval tv = {epoch, 0};
  settimeofday(&tv, nullptr);

  hours = t.tm_hour;
  minutes = t.tm_min;
  snprintf(lastSyncTimeStr, sizeof(lastSyncTimeStr), "%02d:%02d", hours, minutes);
  snprintf(currentDateStr, sizeof(currentDateStr), "%02d.%02d.%02d", t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
  return true;
}

bool updateRTCFromSystemTime() {
  struct tm now;
  if (!getLocalTime(&now)) {
    return false;
  }

  uint8_t sec = decToBcd((uint8_t)now.tm_sec);
  uint8_t min = decToBcd((uint8_t)now.tm_min);
  uint8_t hour = decToBcd((uint8_t)now.tm_hour);
  uint8_t mday = decToBcd((uint8_t)now.tm_mday);
  uint8_t wday = decToBcd((uint8_t)now.tm_wday);
  int fullYear = now.tm_year + 1900;
  uint8_t month = decToBcd((uint8_t)(now.tm_mon + 1));
  uint8_t year = decToBcd((uint8_t)(fullYear % 100));

  // PCF8563
  if (fullYear < 2000) {
    month |= 0x80;
  }

  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write((uint8_t)0x02);
  Wire.write(sec & 0x7F);
  Wire.write(min & 0x7F);
  Wire.write(hour & 0x3F);
  Wire.write(wday & 0x3F);
  Wire.write(mday & 0x07);
  Wire.write(month);
  Wire.write(year);

  return (Wire.endTransmission() == 0);

}

void updateWeatherFromAPI() {
  HTTPClient http;


  http.begin("https://api.open-meteo.com/v1/forecast?latitude=" + String(home_lat) + "&longitude=" + String(home_lon) + "&current=temperature_2m,wind_speed_10m,wind_gusts_10m,wind_direction_10m&hourly=temperature_2m,precipitation,wind_speed_10m&daily=temperature_2m_max,temperature_2m_min,precipitation_sum,wind_speed_10m_max,wind_direction_10m_dominant&timezone=Europe%2FWarsaw");
  if (http.GET() > 0) {
    JsonDocument doc;
    if (!deserializeJson(doc, http.getString())) {
      JsonObject current = doc["current"];
      meteoHome.temperature = current["temperature_2m"];
      meteoHome.windSpeed = current["wind_speed_10m"];
      meteoHome.windGusts = current["wind_gusts_10m"];
      meteoHome.windDirectionDeg = current["wind_direction_10m"];
      meteoHome.valid = true;

      JsonObject daily = doc["daily"];
      JsonArray rainArr = daily["precipitation_sum"];
      if (rainArr.size() > 0) {
        meteoHome.precipitation = rainArr[0] | 0.0;
      }

      JsonObject hourly = doc["hourly"];
      JsonArray timeArr = hourly["time"];
      JsonArray tempArr = hourly["temperature_2m"];
      JsonArray precipArr = hourly["precipitation"];
      JsonArray windArr = hourly["wind_speed_10m"];

      struct tm timeinfo;
      int currentHourIndex = 0;
      if (getLocalTime(&timeinfo)) {
        currentHourIndex = timeinfo.tm_hour;
      }

      for (int i = 0; i < 12; i++) {
        int idx = currentHourIndex + i;
        if (idx < timeArr.size()) {
          hourlyHome.temp[i] = tempArr[idx] | 0.0;
          hourlyHome.precip[i] = precipArr[idx] | 0.0;
          hourlyHome.windSpeed[i] = windArr[idx] | 0.0;
          const char* timeStr = timeArr[idx];
          if (strlen(timeStr) >= 13) {
            hourlyHome.hour[i] = atoi(timeStr + 11);
          } else {
            hourlyHome.hour[i] = (i + hours) % 24;
          }
        }
      }
      hourlyHome.valid = true;

      JsonArray dTimeArr = daily["time"];
      JsonArray tMaxArr = daily["temperature_2m_max"];
      JsonArray tMinArr = daily["temperature_2m_min"];
      JsonArray windArrDaily = daily["wind_speed_10m_max"];
      JsonArray windDirArr = daily["wind_direction_10m_dominant"];

      for (int i = 0; i < 7 && i < dTimeArr.size(); i++) {
        strlcpy(forecastHome.date[i], dTimeArr[i] | "", sizeof(forecastHome.date[i]));
        forecastHome.tMax[i] = tMaxArr[i] | 0.0;
        forecastHome.tMin[i] = tMinArr[i] | 0.0;
        forecastHome.precipitation[i] = rainArr[i] | 0.0;
        forecastHome.windSpeedMax[i] = windArrDaily[i] | 0.0;
        forecastHome.windDirectionMaxDeg[i] = windDirArr[i] | 0;
      }
      forecastHome.valid = true;
    }
  }
  http.end();

  // STOGI (lat=54.36, lon=18.72)
  http.begin("https://api.open-meteo.com/v1/forecast?latitude=" + String(second_lat) + "&longitude=" + String(second_lon) + "&current=temperature_2m,wind_speed_10m,wind_gusts_10m,wind_direction_10m&daily=temperature_2m_max,temperature_2m_min,precipitation_sum,wind_speed_10m_max,wind_direction_10m_dominant&timezone=Europe%2FWarsaw");
  if (http.GET() > 0) {
    JsonDocument doc;
    if (!deserializeJson(doc, http.getString())) {
      JsonObject current = doc["current"];
      meteoSecond.temperature = current["temperature_2m"];
      meteoSecond.windSpeed = current["wind_speed_10m"];
      meteoSecond.windGusts = current["wind_gusts_10m"];
      meteoSecond.windDirectionDeg = current["wind_direction_10m"];
      meteoSecond.valid = true;

      JsonObject daily = doc["daily"];
      JsonArray rainArr = daily["precipitation_sum"];
      if (rainArr.size() > 0) {
        meteoSecond.precipitation = rainArr[0] | 0.0;
      }

      JsonArray timeArr = daily["time"];
      JsonArray tMaxArr = daily["temperature_2m_max"];
      JsonArray tMinArr = daily["temperature_2m_min"];
      JsonArray windArrDaily = daily["wind_speed_10m_max"];
      JsonArray windDirArr = daily["wind_direction_10m_dominant"];

      for (int i = 0; i < 7 && i < timeArr.size(); i++) {
        strlcpy(forecastSecond.date[i], timeArr[i] | "", sizeof(forecastSecond.date[i]));
        forecastSecond.tMax[i] = tMaxArr[i] | 0.0;
        forecastSecond.tMin[i] = tMinArr[i] | 0.0;
        forecastSecond.precipitation[i] = rainArr[i] | 0.0;
        forecastSecond.windSpeedMax[i] = windArrDaily[i] | 0.0;
        forecastSecond.windDirectionMaxDeg[i] = windDirArr[i] | 0;
      }
      forecastSecond.valid = true;
    }
  }
  http.end();
}

const char* getWindDirectionText(int degrees) {
  if (degrees >= 337.5 || degrees < 22.5)  return "N";
  if (degrees >= 22.5  && degrees < 67.5)  return "NE";
  if (degrees >= 67.5  && degrees < 112.5) return "E";
  if (degrees >= 112.5 && degrees < 157.5) return "SE";
  if (degrees >= 157.5 && degrees < 202.5) return "S";
  if (degrees >= 202.5 && degrees < 247.5) return "SW";
  if (degrees >= 247.5 && degrees < 292.5) return "W";
  if (degrees >= 292.5 && degrees < 337.5) return "NW";
  return "-";
}

const char* getDayOfWeekShort(const char* dateStr) {
  if (strlen(dateStr) < 10) return "---";
  
  int year = atoi(dateStr);
  int month = atoi(dateStr + 5);
  int day = atoi(dateStr + 8);

  struct tm t = {0};
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  mktime(&t);

  switch (t.tm_wday) {
    case 0: return "nie";
    case 1: return "pon";
    case 2: return "wto";
    case 3: return "sro";
    case 4: return "czw";
    case 5: return "pią";
    case 6: return "sob";
  }
  return "---";
}

void drawDashboard()
{
  if (currentScreen == 1) {
    drawForecastScreen((String("Prognoza 7 dni: ") + titleHome).c_str(), forecastHome);
    return;
  } else if (currentScreen == 2) {
    drawForecastScreen((String("Prognoza 7 dni: ") + titleSecond).c_str(), forecastSecond);
    return;
  }

  const uint16_t W = display.width();
  const uint16_t H = display.height();
  
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    
    display.drawRect(10, 10, W - 20, H - 20, GxEPD_BLACK);
    display.drawRect(14, 14, W - 28, H - 28, GxEPD_BLACK);

    // Ramka 1: Czas i Wnętrze
    display.drawRoundRect(25, 25, 235, 315, 8, GxEPD_BLACK);
    display.fillRoundRect(27, 27, 231, 28, 6, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&FreeSansBold12pt7b);
    int16_t tbx, tby; uint16_t tbw, tbh;
    const char* t1Title = "STACJA METEO";
    display.getTextBounds(t1Title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(27 + (231 - tbw) / 2 - tbx, 48);
    display.print(t1Title);

    // Ramka 2: Home
    display.drawRoundRect(270, 25, 240, 315, 8, GxEPD_BLACK);
    display.fillRoundRect(272, 27, 236, 28, 6, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&FreeSansBold12pt7b);
    display.getTextBounds(titleHome, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(272 + (236 - tbw) / 2 - tbx, 48);
    display.print(titleHome);

    // Ramka 3: GDANSK STOGI
    display.drawRoundRect(520, 25, 255, 315, 8, GxEPD_BLACK);
    display.fillRoundRect(522, 27, 251, 28, 6, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&FreeSansBold12pt7b);
    display.getTextBounds(titleSecond, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(522 + (251 - tbw) / 2 - tbx, 48);
    display.print(titleSecond);

    // Wyświetlanie godziny w ramce 1
    char timeStr[8];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d", hours, minutes);

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeSansBold24pt7b);
    display.getTextBounds(timeStr, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(25 + (235 - tbw) / 2 - tbx, 95);
    display.print(timeStr);

    display.setFont(&FreeSansBold12pt7b);
    display.getTextBounds(currentDateStr, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(25 + (235 - tbw) / 2 - tbx, 120);
    display.print(currentDateStr);

    display.setFont(&FreeSans9pt7b);
    display.setCursor(35, 150);
    display.print("Odczyt: ");
    display.print(lastSyncTimeStr);

    display.setCursor(35, 175);
    display.print("Temp wew:");
    char intTempBuf[16];
    if (sht4xValid) {
      snprintf(intTempBuf, sizeof(intTempBuf), "%.1f C", internalTemp);
    } else {
      snprintf(intTempBuf, sizeof(intTempBuf), "--.- C");
    }
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(135, 175);
    display.print(intTempBuf);

    display.setFont(&FreeSans9pt7b);
    display.setCursor(35, 205);
    display.print("Wilgotnosc:");
    char intHumBuf[16];
    if (sht4xValid) {
      snprintf(intHumBuf, sizeof(intHumBuf), "%d %%", internalHumidity);
    } else {
      snprintf(intHumBuf, sizeof(intHumBuf), "-- %%");
    }
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(135, 205);
    display.print(intHumBuf);
    display.setFont(&FreeSans9pt7b);
    display.setCursor(35, 240);
    display.print("WiFi: ");
    display.print(wifiLastSuccess ? "OK" : "ERR");
    display.print(" SHT4x: ");
    display.print(sensorReadLastSuccess ? "OK" : "ERR");

    // Buzzer
    display.setFont(&FreeSans9pt7b);
    display.setCursor(35, 270);
    display.print("Buzzer 15min: ");
    display.print(buzzerActive ? "Aktywny" : "Wylaczony");

    // Bateria (Procenty + Napięcie bieżące / min / max)
    display.setFont(&FreeSans9pt7b);
    display.setCursor(35, 298);
    display.print("Bateria:");
    char batBuf[32];
    snprintf(batBuf, sizeof(batBuf), "%d%% (%.2fV)", batteryPercentage, batteryVoltage);
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(35, 323);
    display.print(batBuf);

    // Kolumny pogody bieżącej
    auto drawWeatherColumn = [&](int colX, WeatherData &data) {
      int wy = 80; 
      int labelToValueGap = 32; 
      int rowSpacing = 56;      
      
      display.setFont(&FreeSans9pt7b);
      display.setCursor(colX, wy);
      display.print("Temperatura:");
      
      char valBuf[16];
      display.setFont(&FreeSansBold18pt7b);
      display.setCursor(colX, wy + labelToValueGap);
      if (data.valid) {
        snprintf(valBuf, sizeof(valBuf), "%.1f", data.temperature);
        display.print(valBuf);
        int16_t x1, y1; uint16_t w1, h1;
        display.getTextBounds(valBuf, colX, wy + labelToValueGap, &x1, &y1, &w1, &h1);
        display.setFont(&FreeSans9pt7b);
        display.setCursor(colX + w1 + 4, wy + labelToValueGap);
        display.print("C");
      } else {
        display.print("--.- C");
      }

      wy += rowSpacing;
      display.setFont(&FreeSans9pt7b);
      display.setCursor(colX, wy);
      display.print("Opady (dzis):");
      
      display.setFont(&FreeSansBold18pt7b);
      display.setCursor(colX, wy + labelToValueGap);
      if (data.valid) {
        snprintf(valBuf, sizeof(valBuf), "%.1f", data.precipitation);
        display.print(valBuf);
        int16_t x1, y1; uint16_t w1, h1;
        display.getTextBounds(valBuf, colX, wy + labelToValueGap, &x1, &y1, &w1, &h1);
        display.setFont(&FreeSans9pt7b);
        display.setCursor(colX + w1 + 4, wy + labelToValueGap);
        display.print("mm");
      } else {
        display.print("--.- mm");
      }

      wy += rowSpacing;
      display.setFont(&FreeSans9pt7b);
      display.setCursor(colX, wy);
      display.print("Wiatr:");
      
      display.setFont(&FreeSansBold18pt7b);
      display.setCursor(colX, wy + labelToValueGap);
      if (data.valid) {
        float windVal = convertWind(data.windSpeed);
        snprintf(valBuf, sizeof(valBuf), "%.0f", windVal);
        display.print(valBuf);
        int16_t x1, y1; uint16_t w1, h1;
        display.getTextBounds(valBuf, colX, wy + labelToValueGap, &x1, &y1, &w1, &h1);
        const char* dirText = getWindDirectionText(data.windDirectionDeg);
        char unitAndDir[16];
        const char* unit = (windUnit == 1 ? "kt" : "km/h");
        snprintf(unitAndDir, sizeof(unitAndDir), "%s (%s)", unit, dirText);
        display.setFont(&FreeSans9pt7b);
        display.setCursor(colX + w1 + 4, wy + labelToValueGap);
        display.print(unitAndDir);
      } else {
        display.print("-- km/h");
      }

      wy += rowSpacing;
      display.setFont(&FreeSans9pt7b);
      display.setCursor(colX, wy);
      display.print("Porywy wiatru:");
      
      display.setFont(&FreeSansBold18pt7b);
      display.setCursor(colX, wy + labelToValueGap);
      const char* unit = (windUnit == 1 ? "kt" : "km/h");

      if (data.valid) {
        float gustVal = convertWind(data.windGusts);
        snprintf(valBuf, sizeof(valBuf), "%.0f", gustVal);
        display.print(valBuf);
        int16_t x1, y1; uint16_t w1, h1;
        display.getTextBounds(valBuf, colX, wy + labelToValueGap, &x1, &y1, &w1, &h1);
        display.setFont(&FreeSans9pt7b);
        display.setCursor(colX + w1 + 4, wy + labelToValueGap);
        display.print(unit);
      } else {
        display.print("--");
      }
    };

    drawWeatherColumn(290, meteoHome);
    drawWeatherColumn(540, meteoSecond);

    // ===== DOLNA TABELA GODZINOWA DLA HOME =====
    int chartX = 25;
    int chartY = 345;
    int chartW = 750;
    int chartH = 114;

    display.drawRoundRect(chartX, chartY, chartW, chartH, 6, GxEPD_BLACK);
    display.setFont(&FreeSans9pt7b);
    display.setCursor(chartX + 10, chartY + 18);
    display.print(String(titleHome) + " - tabela na najblizsze 12h");

    if (hourlyHome.valid) {
      display.setFont(&FreeSans9pt7b);
      display.setCursor(chartX + 10, chartY + 42);
      display.print("Godz:");
      display.setCursor(chartX + 10, chartY + 62);
      display.print("Temp:");
      display.setCursor(chartX + 10, chartY + 82);
      display.print("Opady:");
      display.setCursor(chartX + 10, chartY + 102);
      display.print("Wiatr:");

      int dataStartX = chartX + 65;
      int usableW = chartW - 75;
      int colWidth = usableW / 12;

      for (int i = 0; i < 12; i++) {
        int baseCx = dataStartX + (i * colWidth) + 15;
        
        if (i > 0) {
          display.drawFastVLine(dataStartX + (i * colWidth) - 3, chartY + 25, chartH - 28, GxEPD_BLACK);
        }

        char hBuf[8];
        snprintf(hBuf, sizeof(hBuf), "%02d", hourlyHome.hour[i]);
        display.setFont(&FreeSansBold12pt7b);
        display.setCursor(baseCx - 5, chartY + 43);
        display.print(hBuf);

        char tBuf[8];
        snprintf(tBuf, sizeof(tBuf), "%.0fC", hourlyHome.temp[i]);
        display.setFont(&FreeSans9pt7b);
        display.setCursor(baseCx - 5, chartY + 62);
        display.print(tBuf);

        char pBuf[8];
        snprintf(pBuf, sizeof(pBuf), "%.1f", hourlyHome.precip[i]);
        display.setCursor(baseCx, chartY + 82);
        display.print(pBuf);

        char wBuf[8];
        snprintf(wBuf, sizeof(wBuf), "%.0f", hourlyHome.windSpeed[i]);
        display.setCursor(baseCx, chartY + 102);
        display.print(wBuf);
      }
    } else {
      display.setFont(&FreeSans9pt7b);
      display.setCursor(chartX + 20, chartY + 65);
      display.print("Brak danych godzinowych...");
    }

  } while (display.nextPage());
}

void drawForecastScreen(const char* titleText, DailyForecast &forecast)
{
  const uint16_t W = display.width();
  const uint16_t H = display.height();
  
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    
    display.drawRect(10, 10, W - 20, H - 20, GxEPD_BLACK);
    display.drawRect(14, 14, W - 28, H - 28, GxEPD_BLACK);

    display.fillRect(20, 20, W - 40, 45, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&FreeSansBold18pt7b);
    
    int16_t tbx, tby; uint16_t tbw, tbh;
    display.getTextBounds(titleText, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor((W - tbw) / 2 - tbx, 52);
    display.print(titleText);

    display.setFont(&FreeSansBold12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(25, 95);
    display.print("Dzień");
    display.setCursor(120, 95);
    display.print("Data");
    display.setCursor(275, 95);
    display.print("Max/Min");
    display.setCursor(445, 95);
    display.print("Opady");
    display.setCursor(575, 95);
    display.print("Wiatr (max)");

    display.drawFastHLine(20, 105, W - 40, GxEPD_BLACK);

    int startY = 130;
    int rowHeight = 40;

    for (int i = 0; i < 7; i++) {
      int y = startY + (i * rowHeight);

      display.setFont(&FreeSansBold12pt7b);
      display.setCursor(25, y + 25);
      if (forecast.valid && strlen(forecast.date[i]) > 0) {
        display.print(getDayOfWeekShort(forecast.date[i]));
      } else {
        display.print("---");
      }

      display.setFont(&FreeSans9pt7b);
      display.setCursor(110, y + 23);
      if (forecast.valid && strlen(forecast.date[i]) > 0) {
        display.print(forecast.date[i]);
      } else {
        display.print("----.--.--");
      }

      char tempBuf[32];
      if (forecast.valid) {
        snprintf(tempBuf, sizeof(tempBuf), "%.0f/%.0f C", forecast.tMax[i], forecast.tMin[i]);
      } else {
        snprintf(tempBuf, sizeof(tempBuf), "--/-- C");
      }
      display.setFont(&FreeSansBold12pt7b);
      display.setCursor(275, y + 25);
      display.print(tempBuf);

      char rainBuf[32];
      if (forecast.valid) {
        snprintf(rainBuf, sizeof(rainBuf), "%.1f mm", forecast.precipitation[i]);
      } else {
        snprintf(rainBuf, sizeof(rainBuf), "--.- mm");
      }
      display.setCursor(445, y + 25);
      display.print(rainBuf);

      char windBuf[32];
      if (forecast.valid) {
        const char* dirText = getWindDirectionText(forecast.windDirectionMaxDeg[i]);
        float windVal = convertWind(forecast.windSpeedMax[i]);
        const char* unit = (windUnit == 1 ? "kt" : "km/h");
        snprintf(windBuf, sizeof(windBuf), "%.0f %s (%s)", windVal, unit, dirText);

      } else {
        snprintf(windBuf, sizeof(windBuf), "-- km/h");
      }
      display.setCursor(575, y + 25);
      display.print(windBuf);

      if (i < 6) {
        display.drawFastHLine(20, y + 34, W - 40, GxEPD_BLACK);
      }
    }

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeSans9pt7b);
    char footerBuf[64];
    snprintf(footerBuf, sizeof(footerBuf), "reTerminal E1001 | Ekran: %d/3 | Bat Min: %.2fV | Max: %.2fV", currentScreen + 1, batteryMinV, batteryMaxV);
    display.getTextBounds(footerBuf, 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor((W - tbw) / 2 - tbx, H - 16);
    display.print(footerBuf);

  } while (display.nextPage());
}

float convertWind(float kmh) {
    if (windUnit == 1) {
        return kmh / 1.852;   // węzły
    }
    return kmh;               // km/h
}
