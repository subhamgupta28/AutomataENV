
#include "Automata.h"
#include "ArduinoJson.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "config.h"

// const char *HOST = "192.168.29.68";
// int PORT = 8080;

const char *HOST = "raspberry.local";
int PORT = 8010;

Automata automata("ENV", HOST, PORT);
JsonDocument doc;

Adafruit_BMP280 bmp;

Adafruit_NeoPixel led(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

float temp = 0;
float pressure = 0;
int pwm1 = 0;
int pwm2 = 0;
int pwm3 = 0;
bool power = true;
long start = millis();

void action(const Action action)
{
  led.setPixelColor(0, 250, 250, 250);
  led.show();
  int p1 = action.data["pwm1"];
  int p2 = action.data["pwm2"];
  int p3 = action.data["pwm3"];
  if (p1)
    pwm1 = p1;
  if (p2)
    pwm2 = p2;
  if (p3)
    pwm3 = p3;

  delay(100);
  String jsonString;
  serializeJson(action.data, jsonString);
  Serial.println(jsonString);
  led.setPixelColor(0, 0, 250, 0);
  led.show();
}

void sendData()
{

  automata.sendData(doc);
}

void initBMP()
{
  if (!bmp.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void setup()
{
  delay(3000);
  led.begin();
  led.setBrightness(25);
  led.setPixelColor(0, 180, 250, 50);
  led.show();
  delay(300);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(MOSFET_PIN1, OUTPUT);
  pinMode(MOSFET_PIN2, OUTPUT);
  pinMode(MOSFET_PIN3, OUTPUT);

  pinMode(PIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  EEPROM.begin(EEPROM_SIZE);
  pwm1 = EEPROM.read(0);
  pwm2 = EEPROM.read(1);
  pwm3 = EEPROM.read(2);

  analogWrite(MOSFET_PIN1, pwm1);
  analogWrite(MOSFET_PIN2, pwm2);
  analogWrite(MOSFET_PIN3, pwm3);

  initBMP();

  automata.begin();
    JsonDocument doc;
  doc["max"] = 100;
  doc["min"] = 0;
  automata.addAttribute("temp", "Temp", "°C", "DATA|MAIN");
  // automata.addAttribute("temp_c", "Temperature", "C", "DATA|CHART");
  automata.addAttribute("pressure", "Pressure", "PA", "DATA|MAIN");
  // automata.addAttribute("pwm1", "PWM 1", "", "DATA|SLIDER", doc);
  // automata.addAttribute("pwm2", "PWM 2", "", "DATA|SLIDER", doc);
  // automata.addAttribute("pwm3", "PWM 3", "", "DATA|SLIDER", doc);
  automata.addAttribute("button", "Button", "On/Off", "ACTION|MENU|BTN");
  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);
}

void readBMP()
{
  if (bmp.takeForcedMeasurement())
  {
    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
  }
  else
  {
    Serial.println("Forced measurement failed!");
  }
}

void turnOnAll()
{
  if (pwm1 <= 10)
    pwm1 = 255;
  if (pwm2 <= 10)
    pwm2 = 255;
  if (pwm3 <= 10)
    pwm3 = 255;
}
void turnOffAll()
{
  pwm1 = 0;
  pwm2 = 0;
  pwm3 = 0;
}

void loop()
{
  automata.loop();

  readBMP();
  doc["temp"] = temp;
  // doc["temp_c"] = temp;
  // doc["pwm1"] = pwm1;
  // doc["pwm2"] = pwm2;
  // doc["pwm3"] = pwm3;
  doc["pressure"] = String(pressure/1000, 2);
  doc["button"] = digitalRead(BUTTON);

  if ((millis() - start) > 1000)
  {
    automata.sendLive(doc);
    start = millis();
  }

  if (digitalRead(BUTTON) == LOW)
  {
    JsonDocument doc;
    doc["button"] = digitalRead(BUTTON);
    doc["key"] = "button";
    automata.sendAction(doc);
    power = !power;
    if (power)
    {
      led.setPixelColor(0, 250, 0, 0);
      led.show();
      turnOnAll();
    }
    else
    {
      led.setPixelColor(0, 0, 250, 0);
      led.show();
      turnOffAll();
    }
    delay(200);
  }

  // analogWrite(MOSFET_PIN1, pwm2);
  // analogWrite(MOSFET_PIN2, pwm1);
  // analogWrite(MOSFET_PIN3, pwm3);

  delay(100);
  led.setPixelColor(0, 0, 0, 0);
  led.show();
}
