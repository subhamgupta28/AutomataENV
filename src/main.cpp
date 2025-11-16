
#include "Automata.h"
#include "Arduino.h"
#include "ArduinoJson.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "config.h"
#include "tvoc_sensor.h"
#include <BH1750.h>

const char *HOST = "automata.realsubhamgupta.in";
int PORT = 443;

// const char *HOST = "raspberry.local";
// int PORT = 8010;
#define MPM10_I2C_ADDR 0x4D
BH1750 lightMeter;
Automata automata("ENV", HOST, PORT,"0.tcp.in.ngrok.io", 14730);
JsonDocument doc;
Adafruit_AHTX0 aht;

Adafruit_NeoPixel led(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

float temp = 0;
float pressure = 0;
int iaqScore = 0;
String aqiStatus;
bool power = true;
long start = millis();
bool displayOnOff = true;
void action(const Action action)
{
  if (action.data.containsKey("displayOnOff"))
  {
    displayOnOff = action.data["displayOnOff"];
  }else{

  }
  led.setPixelColor(0, 250, 250, 250);
  led.show();

  delay(100);

  led.setPixelColor(0, 0, 250, 0);
  led.show();
}

void sendData()
{

  automata.sendData(doc);
}
uint16_t readPMRegister(uint8_t regHigh, uint8_t regLow)
{
  Wire.beginTransmission(MPM10_I2C_ADDR);
  Wire.write(regHigh);         // start at high-byte register
  Wire.endTransmission(false); // repeated start
  Wire.requestFrom(MPM10_I2C_ADDR, (uint8_t)2);

  if (Wire.available() == 2)
  {
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    return ((uint16_t)high << 8) | low;
  }
  return 0;
}

String airQuality(float pm25)
{
  if (pm25 <= 12.0)
    return "Good 🌿";
  else if (pm25 <= 35.4)
    return "Moderate 🙂";
  else if (pm25 <= 55.4)
    return "Unhealthy for Sensitive 😷";
  else if (pm25 <= 150.4)
    return "Unhealthy 😨";
  else if (pm25 <= 250.4)
    return "Very Unhealthy ☠️";
  else
    return "Hazardous 🔥";
}

void setup()
{

  delay(500);
  led.begin();
    Serial.begin(115200);
  led.setBrightness(25);
  led.setPixelColor(0, 180, 250, 50);
  led.show();
  delay(300);
  led.setPixelColor(0, 0, 0, 250);
  led.show();
  delay(300);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  lightMeter.begin();
  if (aht.begin())
  {
    Serial.println("Found AHT20");
  }
  tvoc_init();                   // Initialize TVOC sensor
  tvoc_set_device_active_mode(); // Set it to active mode
  automata.begin();

  JsonDocument doc;
  doc["max"] = 100;
  doc["min"] = 0;
  automata.addAttribute("temp", "Temp", "C", "DATA|MAIN");
  automata.addAttribute("humid", "Humidity", "%", "DATA|MAIN", doc);
  automata.addAttribute("lux", "Light", "lx", "DATA|MAIN");
  automata.addAttribute("tvoc", "TVOC", "mg/m³", "DATA|AUX");
  automata.addAttribute("co2", "CO2", "ppm", "DATA|MAIN");
  automata.addAttribute("ch2o", "CH2O", "ppb", "DATA|AUX");

  automata.addAttribute("tvocStatus", "TVOC", "", "DATA|AUX");
  automata.addAttribute("co2Status", "CO2", "", "DATA|AUX");
  automata.addAttribute("ch2oStatus", "CH2O", "", "DATA|AUX");
  automata.addAttribute("pm1", "PM 1.0", "µg/m³", "DATA|AUX");
  automata.addAttribute("pm25", "PM 2.5", "µg/m³", "DATA|MAIN");
  automata.addAttribute("pm10", "PM 10", "µg/m³", "DATA|AUX");
  automata.addAttribute("aqi", "AQI", "", "DATA|MAIN");
  automata.addAttribute("quality", "GAS Status", "", "DATA|MAIN");
  automata.addAttribute("aqiStatus", "AQI Status", "", "DATA|MAIN");

  automata.addAttribute("pm03count", "PM 3.0", "pcs/0.1L", "DATA|AUX");
  automata.addAttribute("pm05count", "PM 2.5", "pcs/0.1L", "DATA|AUX");
  automata.addAttribute("pm10count", "PM 1.0", "pcs/0.1L", "DATA|AUX");
  automata.addAttribute("pm25count", "PM 2.5", "pcs/0.1L", "DATA|AUX");
  automata.addAttribute("pm50count", "PM 5.0", "pcs/0.1L", "DATA|AUX");
  automata.addAttribute("pm100count", "PM 10.0", "pcs/0.1L", "DATA|AUX");
  // automata.addAttribute("pwm1", "PWM 1", "", "DATA|SLIDER", doc);
  // automata.addAttribute("pwm2", "PWM 2", "", "DATA|SLIDER", doc);
  // automata.addAttribute("pwm3", "PWM 3", "", "DATA|SLIDER", doc);
  automata.addAttribute("button", "Button", "On/Off", "ACTION|MENU|BTN");
  automata.addAttribute("displayOnOff", "Display Power", "W", "ACTION|MENU|SWITCH");
  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);
  led.setPixelColor(0, 255, 0, 0);
  led.show();
  delay(200);
}
String evaluateAirQuality(float pm1, float pm25, float pm10, float co2, float tvoc, float ch2o, float humidity)
{
  float score = 0.0;

  // // --- PM1.0 score ---
  // if (pm1 > 35.0)
  //   score += 1.0;
  // else if (pm1 > 12.0)
  //   score += 0.5;

  // // --- PM2.5 score ---
  // if (pm25 > 35.0)
  //   score += 1.0;
  // else if (pm25 > 12.0)
  //   score += 0.5;

  // // --- PM10 score ---
  // if (pm10 > 150.0)
  //   score += 1.0;
  // else if (pm10 > 50.0)
  //   score += 0.5;

  // --- CO2 score ---
  if (co2 > 1200)
    score += 1.0;
  else if (co2 > 800)
    score += 0.5;

  // --- TVOC score ---
  if (tvoc > 0.6)
    score += 1.0;
  else if (tvoc > 0.3)
    score += 0.5;

  // --- CH2O (formaldehyde) score ---
  if (ch2o > 120)
    score += 1.0;
  else if (ch2o > 80)
    score += 0.5;

  // --- Humidity comfort range (30–60%) ---
  if (humidity < 30.0 || humidity > 60.0)
    score += 0.5;

  // --- Determine final air quality category ---
  if (score <= 1.0)
    return "Excellent 🌿";
  else if (score <= 2.5)
    return "Moderate 🙂";
  else if (score <= 4.0)
    return "Unhealthy 😷";
  else
    return "Hazardous ☠️";
}

// --- Calculate AQI using US EPA breakpoints ---
float calculateAQI(float pm25, float pm10)
{
  // AQI breakpoints (simplified US EPA version)
  struct AQIBreakpoint
  {
    float concLow, concHigh;
    int aqiLow, aqiHigh;
  };

  // PM2.5 breakpoints
  AQIBreakpoint pm25Table[] = {
      {0.0, 12.0, 0, 50},
      {12.1, 35.4, 51, 100},
      {35.5, 55.4, 101, 150},
      {55.5, 150.4, 151, 200},
      {150.5, 250.4, 201, 300},
      {250.5, 350.4, 301, 400},
      {350.5, 500.4, 401, 500},
  };

  // PM10 breakpoints
  AQIBreakpoint pm10Table[] = {
      {0, 54, 0, 50},
      {55, 154, 51, 100},
      {155, 254, 101, 150},
      {255, 354, 151, 200},
      {355, 424, 201, 300},
      {425, 504, 301, 400},
      {505, 604, 401, 500},
  };

  auto calcAQI = [](float conc, AQIBreakpoint *table, int size)
  {
    for (int i = 0; i < size; i++)
    {
      if (conc >= table[i].concLow && conc <= table[i].concHigh)
      {
        return ((table[i].aqiHigh - table[i].aqiLow) / (table[i].concHigh - table[i].concLow)) *
                   (conc - table[i].concLow) +
               table[i].aqiLow;
      }
    }
    return 500.0f; // max AQI
  };

  float aqi25 = calcAQI(pm25, pm25Table, 7);
  float aqi10 = calcAQI(pm10, pm10Table, 7);

  return max(aqi25, aqi10); // overall AQI is the worse of the two
}
String getAQIStatus(float aqi)
{
  if (aqi <= 50)
    return "Good 🌿";
  else if (aqi <= 100)
    return "Moderate 🙂";
  else if (aqi <= 150)
    return "Sensitive 😷";
  else if (aqi <= 200)
    return "Unhealthy 😨";
  else if (aqi <= 300)
    return "Very Unhealthy ☠️";
  else
    return "Hazardous 🔥";
}
void loop()
{

  sensors_event_t humidity, temp;
  float lux = lightMeter.readLightLevel();
  aht.getEvent(&humidity, &temp);
  doc["temp"] = String(temp.temperature, 2);
  doc["lux"] = String(lux);
  doc["humid"] = String(humidity.relative_humidity, 2);
  TvocSensorData tvocData = tvoc_get_active_device_data();
  uint16_t pm1 = readPMRegister(0x24, 0x25);
  uint16_t pm25 = readPMRegister(0x20, 0x21);
  uint16_t pm10 = readPMRegister(0x22, 0x23);
  // Additional PM counts
  uint16_t pm03count = readPMRegister(0x26, 0x27);
  uint16_t pm05count = readPMRegister(0x28, 0x29);
  uint16_t pm10count = readPMRegister(0x2A, 0x2B);
  uint16_t pm25count = readPMRegister(0x2C, 0x2D);
  uint16_t pm50count = readPMRegister(0x2E, 0x2F);
  uint16_t pm100count = readPMRegister(0x30, 0x31);
  float aqiValue = calculateAQI(pm25, pm10);
  doc["aqi"] = String(aqiValue, 2);
  doc["aqiStatus"] = getAQIStatus(aqiValue);
  doc["pm1"] = pm1;
  doc["pm25"] = pm25;
  doc["pm10"] = pm10;
  doc["pm03count"] = pm03count;
  doc["pm05count"] = pm05count;
  doc["pm10count"] = pm10count;
  doc["pm25count"] = pm25count;
  doc["pm50count"] = pm50count;
  doc["pm100count"] = pm100count;
  doc["displayOnOff"] = displayOnOff;
  if (tvocData.valid)
  {
    doc["tvoc"] = String(tvocData.tvoc, 3);
    doc["tvocStatus"] = tvocData.tvoc_status;
    doc["co2"] = tvocData.co2;
    doc["co2Status"] = tvocData.co2_status;
    doc["ch2o"] = tvocData.ch2o;
    doc["ch2oStatus"] = tvocData.ch2o_status;
    doc["quality"] = evaluateAirQuality(
        pm1,
        pm25,
        pm10,
        tvocData.co2,
        tvocData.tvoc,
        tvocData.ch2o,
        humidity.relative_humidity);
  }

  if (displayOnOff)
  {
    if (aqiValue <= 50)
      led.setPixelColor(0, 0, 255, 0); // Green
    else if (aqiValue <= 100)
      led.setPixelColor(0, 120, 255, 0); // Yellow
    else if (aqiValue <= 150)
      led.setPixelColor(0, 255, 120, 0); // Orange
    else if (aqiValue <= 200)
      led.setPixelColor(0, 255, 0, 0); // Red
    else
      led.setPixelColor(0, 180, 0, 255); // Purple (very bad)
    led.show();
  }

  doc["button"] = digitalRead(BUTTON);

  if ((millis() - start) > 1000)
  {
    if (displayOnOff)
    {
      led.setPixelColor(0, 50, 50, 50);
      led.show();
    }

    automata.sendLive(doc);
    start = millis();
  }

  if (digitalRead(BUTTON) == LOW)
  {
    JsonDocument doc;
    doc["button"] = digitalRead(BUTTON);
    doc["key"] = "button";
    automata.sendAction(doc);
    if (displayOnOff)
    {
      led.setPixelColor(0, 250, 50, 50);
      led.show();
    }

    delay(200);
  }

  delay(100);
  if (!displayOnOff)
  {
    led.setPixelColor(0, 0, 0, 0);
    led.show();
  }
}
