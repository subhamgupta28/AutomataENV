
#include "Automata.h"
#include "ArduinoJson.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "config.h"
#include "tvoc_sensor.h"
#include <BH1750.h>


// const char *HOST = "192.168.1.35";
// int PORT = 8010;

const char *HOST = "raspberry.local";
int PORT = 8010;

BH1750 lightMeter;
Automata automata("ENV", HOST, PORT);
JsonDocument doc;
Adafruit_AHTX0 aht;

Adafruit_NeoPixel led(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

float temp = 0;
float pressure = 0;
int iaqScore = 0;
String aqiStatus;
bool power = true;
long start = millis();

void action(const Action action)
{
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

void setup()
{
  delay(500);
  led.begin();
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
  automata.addAttribute("humid", "Humidity", "%", "DATA|GAUGE", doc);
  automata.addAttribute("lux", "Light", "lx", "DATA|MAIN");
  automata.addAttribute("tvoc", "TVOC", "ppm", "DATA|MAIN");
  automata.addAttribute("co2", "CO2", "ppm", "DATA|MAIN");
  automata.addAttribute("ch2o", "CH2O", "ppb", "DATA|MAIN");
  automata.addAttribute("iaq_score", "IAQ Score", "", "DATA|AUX");
  automata.addAttribute("aqi", "AQI", "", "DATA|MAIN");
  automata.addAttribute("tvocStatus", "TVOC", "", "DATA|AUX");
  automata.addAttribute("co2Status", "CO2", "", "DATA|AUX");
  automata.addAttribute("ch2oStatus", "CH2O", "", "DATA|AUX");
  // automata.addAttribute("pwm1", "PWM 1", "", "DATA|SLIDER", doc);
  // automata.addAttribute("pwm2", "PWM 2", "", "DATA|SLIDER", doc);
  // automata.addAttribute("pwm3", "PWM 3", "", "DATA|SLIDER", doc);
  automata.addAttribute("button", "Button", "On/Off", "ACTION|MENU|BTN");
  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);
  led.setPixelColor(0, 255, 0, 0);
  led.show();
  delay(200);
}

void aqiRead(TvocSensorData tvocData)
{
  if (tvocData.tvoc <= 220)
  {
    iaqScore += 0;
  }
  else if (tvocData.tvoc <= 660)
  {
    iaqScore += 1;
  }
  else if (tvocData.tvoc <= 2200)
  {
    iaqScore += 2;
  }
  else if (tvocData.tvoc <= 5500)
  {
    iaqScore += 3;
  }
  else
  {
    iaqScore += 4;
  }

  // CO2 contribution
  if (tvocData.co2 <= 1000)
  {
    iaqScore += 0;
  }
  else if (tvocData.co2 <= 2000)
  {
    iaqScore += 1;
  }
  else
  {
    iaqScore += 2;
  }

  // CH2O contribution
  if (tvocData.ch2o <= 80)
  {
    iaqScore += 0;
  }
  else if (tvocData.ch2o <= 200)
  {
    iaqScore += 1;
  }
  else
  {
    iaqScore += 2;
  }

  // Map IAQ score to a qualitative AQI status

  if (iaqScore <= 1)
  {
    aqiStatus = "Excellent";
  }
  else if (iaqScore <= 3)
  {
    aqiStatus = "Good";
  }
  else if (iaqScore <= 5)
  {
    aqiStatus = "Moderate";
  }
  else if (iaqScore <= 7)
  {
    aqiStatus = "Poor";
  }
  else
  {
    aqiStatus = "Very Poor";
  }
  doc["iaq_score"] = iaqScore;
  doc["aqi"] = aqiStatus;
}

void loop()
{
  automata.loop();

  sensors_event_t humidity, temp;
  float lux = lightMeter.readLightLevel();
  aht.getEvent(&humidity, &temp);
  doc["temp"] = String(temp.temperature, 2);
  doc["lux"] = String(lux);
  doc["humid"] = String(humidity.relative_humidity, 2);
  TvocSensorData tvocData = tvoc_get_active_device_data();

  if (tvocData.valid)
  {
    doc["tvoc"] = String(tvocData.tvoc, 3);
    doc["tvocStatus"] = tvocData.tvoc_status;
    doc["co2"] = tvocData.co2;
    doc["co2Status"] = tvocData.co2_status;
    doc["ch2o"] = tvocData.ch2o;
    doc["ch2oStatus"] = tvocData.ch2o_status;
    aqiRead(tvocData);
  }

  doc["button"] = digitalRead(BUTTON);

  if ((millis() - start) > 1000)
  {
    led.setPixelColor(0, 10, 0, 10);
    led.show();
    automata.sendLive(doc);
    start = millis();
  }

  if (digitalRead(BUTTON) == LOW)
  {
    JsonDocument doc;
    doc["button"] = digitalRead(BUTTON);
    doc["key"] = "button";
    automata.sendAction(doc);

    led.setPixelColor(0, 250, 50, 50);
    led.show();
    delay(200);
  }

  delay(100);
  led.setPixelColor(0, 0, 0, 0);
  led.show();
}
