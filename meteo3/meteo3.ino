/******************************************************************
  Created with PROGRAMINO IDE for Arduino - 06.04.2019 21:46:06
  Project     :
  Libraries   :
  Author      :
  Description :
******************************************************************/

//libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//DHT11 - temperature & humidity sensor
#define DHTPIN 7
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//BMP280 - temperatue, pressure and altitude sensor
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK); //using software SPI

//LCD I2C - LiquidCrystal
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
const uint8_t LCD_COLUNNS = 16;
const uint8_t LCD_ROWS = 2;

//RGB diode
#define RED_RGB_PIN 3
#define GREEN_RGB_PIN 5
#define BLUE_RGB_PIN 6

//tactile switch
const uint8_t BUTTON_PIN = 2;

//program logic
const char DEGREE_SIGN = 223;
const int BOUNCE_TIME = 200; //in miliseconds
const int DEFAULT_BAUD_RATE = 9600;
const int SERIAL_PRINT_DELAY = 2000; //in miliseconds
const int PRESSURE_CALIBRATION_DIFFERENCE = 26;

bool printOnSerial = false;

class Mode {
  private:
    String header; //e.g. "TEMPERATURE"
    String unitLCD; //e.g. "hPa"
    String unit;
    uint8_t decimalDigits; //float value can be converted to int if the decimal part is unnecessary
    uint8_t headerShift; //char count from the left of the screen by which the header will be shifted
    uint8_t valueShift; //char count from the left of the screen by which the value will be shifted

  public:
    Mode(String header, String unitLCD, String unit, uint8_t decimalDigits, uint8_t headerShift, uint8_t valueShift)
      : header(header), unitLCD(unitLCD), unit(unit), decimalDigits(decimalDigits), headerShift(headerShift), valueShift(valueShift) { }

    void displayDataOnScreen() {
      lcd.clear();
      lcd.setCursor(headerShift, 0);
      lcd.print(header);
      lcd.setCursor(valueShift, 1);
      lcd.print(getValue(), decimalDigits);
      lcd.print(" ");
      lcd.print(unitLCD);
    }

    void updateDataOnScreen() {
      lcd.setCursor(valueShift, 1);
      lcd.print(getValue(), decimalDigits);
      lcd.print(" ");
      lcd.print(unitLCD);
      lcd.print(" "); //necessary for situations where we go from single digit reading to 2 digit reading
    }

    void printDataOnSerial() {
      //      Serial.print("*M"); //begins the string display on the bt app
      Serial.print(header);
      Serial.print(": ");
      Serial.print(getValue(), decimalDigits);
      Serial.print(" ");
      Serial.println(unit);
      //      Serial.println('*'); //ends the string display on the bt app
    }

    virtual inline float getValue() = 0;
};

class TemperatureMode : public Mode {
  public:
    using Mode::Mode;
    inline float getValue() override  {
      return dht.readTemperature();
    }
};

class HumidityMode : public Mode {
  public:
    using Mode::Mode;
    inline float getValue() override {
      return dht.readHumidity();
    }
};

class HeatIndexMode : public Mode {
  public:
    using Mode::Mode;
    inline float getValue() override  {
      return dht.computeHeatIndex(dht.readTemperature(), dht.readHumidity(), false);
    }
};

class PressureMode : public Mode {
  public:
    using Mode::Mode;
    inline float getValue() override  {
      return bme.readPressure() / 100 + PRESSURE_CALIBRATION_DIFFERENCE;
    }

};


const short MODES_COUNT = 3;

Mode* temperature = new TemperatureMode("TEMPERATURE", String(String(DEGREE_SIGN) + "C"), "°C", 1, 2, 4);
Mode* heatIndex = new HeatIndexMode("HEAT INDEX", String(String(DEGREE_SIGN) + "C"), "°C", 1, 3, 4);
Mode* pressure = new PressureMode("PRESSURE", "hPa", "hPa", 0, 4, 4);

Mode* modes[MODES_COUNT] = {temperature, heatIndex, pressure};

uint8_t currentModeIndex = 0;
uint8_t oldModeIndex = 0;




//functions
inline void nextMode() {
  if (currentModeIndex == MODES_COUNT - 1) { //we are at the last mode, so we have to loop back to the beginning
    currentModeIndex = 0;
  } else {
    currentModeIndex++;
  }
}

inline void previousMode() {
  if (currentModeIndex == 0) { //we are at the first mode, so we have to loop back to the end
    currentModeIndex = MODES_COUNT - 1;
  } else {
    currentModeIndex--;
  }
}

//Interrupt Service Routine
void buttonHandler()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  //if interrupts come faster than BOUNCE_TIME, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > BOUNCE_TIME) {
    nextMode();
  }
  last_interrupt_time = interrupt_time;
}

void serialInputHandler() {
  char incomingCharacter = Serial.read();
  switch (incomingCharacter) {
    case 'n':
      nextMode();
      modes[currentModeIndex]->printDataOnSerial();
      break;
    case 'p':
      previousMode();
      modes[currentModeIndex]->printDataOnSerial();
      break;
    case 'e':
      printOnSerial = true;
      break;
    case 'd':
      printOnSerial = false;
      break;
    case 'o':
      modes[currentModeIndex]->printDataOnSerial();
      break;
  }
}

void printModeDataOnSerial() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  //if interrupts come faster than BOUNCE_TIME, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > SERIAL_PRINT_DELAY) {
    modes[currentModeIndex]->printDataOnSerial();
    last_interrupt_time = interrupt_time;
  }
}

void setColor(uint8_t red, uint8_t green, uint8_t blue)
{
  analogWrite(RED_RGB_PIN, 255 - red);
  analogWrite(GREEN_RGB_PIN, 255 - green);
  analogWrite(BLUE_RGB_PIN, 255 - blue);
}

void humidityHandler() {
  static HumidityMode h("HUMIDITY", "%", "%", 0, 2, 4);
  if (h.getValue() < 30.0) {
    setColor(0, 0, 255);
  }
  else if (h.getValue() > 55.0) {
    setColor(255, 0, 0);
  }
  else {
    setColor(0, 255, 0);
  }
}


void setup()
{
  Serial.begin(DEFAULT_BAUD_RATE);
  pinMode(BUTTON_PIN, INPUT_PULLUP); //INPUT_PULLUP should provide better handling for interrupts
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonHandler, FALLING); // Attaching the ISR (Interrupt Service Routine) to the interrupt (INT0 in this case)
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (!bme.begin()) { //check if BMP280 is connected and initialize it
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
  dht.begin(); //initialize the DHT sensor
  lcd.begin(LCD_COLUNNS, LCD_ROWS); //Defining 16 columns and 2 rows of lcd display
  lcd.backlight(); //enables the backlight on the LCD
  modes[currentModeIndex]->displayDataOnScreen();
}


void loop() {
  if (oldModeIndex != currentModeIndex) {
    modes[currentModeIndex]->displayDataOnScreen();
    oldModeIndex = currentModeIndex;
  } else {
    modes[currentModeIndex]->updateDataOnScreen();
  }
  if (Serial.available() > 0) {
    serialInputHandler();
  }
  if (printOnSerial) {
    printModeDataOnSerial();
  }
  humidityHandler();
  delay(200); //don't try to redraw too often
}
