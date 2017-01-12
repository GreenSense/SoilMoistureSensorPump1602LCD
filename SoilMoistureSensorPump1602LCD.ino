#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

long second = 1000;
long minute = second * 60;
long hour = minute * 60;

long lastReadingTime = 0;

//long readingInterval = 1 * hour;
//long readingInterval = 5 * minute;
long readingInterval = 3 * second;

#define moisturePin 0
#define sensorPowerPin 12
#define button1Pin 3
#define button2Pin 4
#define button3Pin 5
#define button4Pin 6
#define button5Pin 7
#define button6Pin 8
#define button7Pin 9
#define button8Pin 10
#define pumpPin 11

int moistureLevel = 0;
int moistureLevelRaw = 0;

bool reverseSoilMoistureSensor = false;
int wetReading = (reverseSoilMoistureSensor ? 0 : 1024);
int dryReading = (reverseSoilMoistureSensor ? 1024 : 0);

int dryReadingAddress = 0;
int wetReadingAddress = 1;

long lastDebounceTime = 0;
long debounceDelay = 200;

int threshold = 30;

bool pumpIsOn = 0;
long pumpStartTime = 0;
long lastPumpFinishTime = 0;
int pumpBurstDuration = 1 * second;
long pumpWaitOffDuration = 5 * second;
//long pumpWaitOffDuration = 30*second;
//long pumpWaitOffDuration = 2*minute;

bool sensorIsOn = true;
long lastSensorOnTime = 0;
int delayAfterTurningSensorOn = 3 * second;

bool screenBacklightIsOn = true;
long lastDisplayTime = 0;
long displayRefreshInterval = 1000;

bool buttonOnValue = 0; // 1 if button is active HIGH and 0 if button is active LOW

#define CALIBRATION_MODE_OFF 0
#define CALIBRATION_MODE_DRY 1
#define CALIBRATION_MODE_WET 2

long calibrationTriggerTime = 0;
long calibrationInterval = 5000;
int calibrationMode = CALIBRATION_MODE_OFF;

long lastSerialOutputTime = 0;
long serialOutputInterval = readingInterval;

bool isDebug = false;

#define SERIAL_MODE_CSV 1
#define SERIAL_MODE_QUERYSTRING 2

int serialMode = SERIAL_MODE_QUERYSTRING;

#define PUMP_STATUS_OFF 0
#define PUMP_STATUS_ON 1
#define PUMP_STATUS_AUTO 2

int pumpStatus = PUMP_STATUS_AUTO;

void setup()
{
  Serial.begin(9600);

  if (isDebug)
    Serial.println("Starting irrigator");

  pinMode(pumpPin, OUTPUT);
  pinMode(sensorPowerPin, OUTPUT);

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);
  pinMode(button4Pin, INPUT_PULLUP);
  pinMode(button5Pin, INPUT_PULLUP);
  pinMode(button6Pin, INPUT_PULLUP);
  pinMode(button7Pin, INPUT_PULLUP);
  pinMode(button8Pin, INPUT_PULLUP);

  if (EEPROM.read(dryReadingAddress) != 0)
    dryReading = getDryReading();
  else
    setDryReading(dryReading);

  if (EEPROM.read(wetReadingAddress) != 0)
    wetReading = getWetReading();
  else
    setWetReading(wetReading);

  lcd.init();

  lcd.backlight();

  // If the interval is less than 2 seconds turn the sensor on and leave it on (otherwise it will be turned on just before it's needed)
  if (readingInterval <= delayAfterTurningSensorOn)
  {
    sensorOn();

    // TODO: Remove if not needed
    //delay(delayAfterTurningSensorOn);
  }

}

void loop()
{
  checkCommand();

  checkButton();

  checkCalibrationTimeout();

  takeReading();

  serialPrintData();

  displayReading();

  irrigateIfNeeded();

  delay(1);
}

/* Commands */
void checkCommand()
{
  // TODO: Is this function still required now that calibration is completed using buttons?
  while (Serial.available() > 0)
  {
    char command = Serial.read();

    Serial.println(command);

    switch (command)
    {
      case 'D':
        lastReadingTime = 0;
        takeReading();
        setDryReading(moistureLevelRaw);
        break;
      case 'W':
        lastReadingTime = 0;
        takeReading();
        setWetReading(moistureLevelRaw);
        break;
      case 'N':
        Serial.println("Turning pump on");
        pumpStatus = PUMP_STATUS_ON;
        pumpOn();
        break;
      case 'F':
        Serial.println("Turning pump off");
        pumpStatus = PUMP_STATUS_OFF;
        pumpOff();
        break;
      case 'A':
        Serial.println("Turning pump to auto");
        pumpStatus = PUMP_STATUS_AUTO;
        irrigateIfNeeded();
        break;
    }
  }
}

/* Buttons */
void checkButton()
{
  if (lastDebounceTime + debounceDelay < millis()
      || lastDebounceTime == 0) {

    int reading1 = digitalRead(button1Pin);
    int reading2 = digitalRead(button2Pin);
    int reading3 = digitalRead(button3Pin);
    int reading4 = digitalRead(button4Pin);
    int reading5 = digitalRead(button5Pin);
    int reading6 = digitalRead(button6Pin);
    int reading7 = digitalRead(button7Pin);
    int reading8 = digitalRead(button8Pin);

    if (reading1 == buttonOnValue)
      button1Pressed();

    if (reading2 == buttonOnValue)
      button2Pressed();

    if (reading3 == buttonOnValue)
      button3Pressed();

    if (reading4 == buttonOnValue)
      button4Pressed();

    if (reading5 == buttonOnValue)
      button5Pressed();

    if (reading6 == buttonOnValue)
      button6Pressed();

    if (reading7 == buttonOnValue)
      button7Pressed();

    if (reading8 == buttonOnValue)
      button8Pressed();

    if (reading1 == buttonOnValue
        || reading2 == buttonOnValue
        || reading3 == buttonOnValue
        || reading4 == buttonOnValue
        || reading5 == buttonOnValue
        || reading6 == buttonOnValue
        || reading7 == buttonOnValue
        || reading8 == buttonOnValue
       )
    {

      lastDebounceTime = millis();


      if (isDebug)
        Serial.println("Button pressed");

      // Reset the last reading time to force another reading
      lastReadingTime = 0; // TODO: Check if needed
    }
  }
}


void button1Pressed()
{
  if (isDebug)
    Serial.println("Button 1 pressed");

  if (calibrationMode == CALIBRATION_MODE_OFF)
  {
    threshold --;
    if (isDebug)
    {
      Serial.print("Threshold: " );
      Serial.println(threshold);
    }
  }
  else if (calibrationMode == CALIBRATION_MODE_DRY && calibrationTriggerTime + calibrationInterval > millis())
  {
    dryReading--;
    if (isDebug)
    {
      Serial.print("Dry reading: " );
      Serial.println(dryReading);
    }
    // Reset the timer
    calibrationTriggerTime = millis();
  }
  else if (calibrationMode == CALIBRATION_MODE_WET && calibrationTriggerTime + calibrationInterval > millis())
  {
    wetReading--;
    if (isDebug)
    {
      Serial.print("Wet reading: " );
      Serial.println(wetReading);
    }
    // Reset the timer
    calibrationTriggerTime = millis();
  }

  refreshDisplay();
}

void button2Pressed()
{
  if (isDebug)
    Serial.println("Button 2 pressed");

  if (calibrationMode == CALIBRATION_MODE_OFF)
  {
    threshold ++;
    if (isDebug)
    {
      Serial.print("T: " );
      Serial.println(threshold);
    }
  }
  else if (calibrationMode == CALIBRATION_MODE_DRY && calibrationTriggerTime + calibrationInterval > millis())
  {
    dryReading++;
    if (isDebug)
    {
      Serial.print("Dry reading: " );
      Serial.println(dryReading);
    }
    // Reset the timer
    calibrationTriggerTime = millis();
  }
  else if (calibrationMode == CALIBRATION_MODE_WET && calibrationTriggerTime + calibrationInterval > millis())
  {
    wetReading++;
    if (isDebug)
    {
      Serial.print("Wet reading: " );
      Serial.println(wetReading);
    }
    // Reset the timer
    calibrationTriggerTime = millis();
  }

  refreshDisplay();
}

void button3Pressed()
{
  if (isDebug)
    Serial.println("Button 3 pressed");

  if (calibrationMode == CALIBRATION_MODE_OFF)
    initiateDryCalibration();
  else if (calibrationMode == CALIBRATION_MODE_DRY && calibrationTriggerTime + calibrationInterval > millis())
    confirmCalibrateDry();
  else
    cancelCalibration();
}

void button4Pressed()
{
  if (isDebug)
    Serial.println("Button 4 pressed");

  if (calibrationMode == CALIBRATION_MODE_OFF)
    initiateWetCalibration();
  else if (calibrationMode == CALIBRATION_MODE_WET && calibrationTriggerTime + calibrationInterval > millis())
    confirmCalibrateWet();
  else
    cancelCalibration();
}

void button5Pressed()
{
  if (isDebug)
    Serial.println("Button 5 pressed");

  toggleScreenBacklight();
}

void button6Pressed()
{
  if (isDebug)
    Serial.println("Button 6 pressed");

  refreshDisplay();
}

void button7Pressed()
{
  if (isDebug)
    Serial.println("Button 7 pressed");


  refreshDisplay();
}

void button8Pressed()
{
  if (isDebug)
    Serial.println("Button 8 pressed");


  refreshDisplay();
}

/* Readings */
void takeReading()
{
  if (lastReadingTime + readingInterval < millis()
      || lastReadingTime == 0)
  {
    if (!sensorIsOn && readingInterval > delayAfterTurningSensorOn)
    {
      sensorOn();
    }
    else if (sensorIsOn && lastSensorOnTime + delayAfterTurningSensorOn < millis()
             || readingInterval < delayAfterTurningSensorOn)
    {
      if (isDebug)
        Serial.println("Preparing to take reading");

      lastReadingTime = millis();

      int readingSum  = 0;
      int totalReadings = 10;

      for (int i = 0; i < totalReadings; i++)
      {
        int reading = analogRead(moisturePin);

        readingSum += reading;
      }

      int averageReading = readingSum / totalReadings;

      moistureLevelRaw = averageReading;

      moistureLevel = calculateMoistureLevel(averageReading);

      if (moistureLevel < 0)
        moistureLevel = 0;

      if (moistureLevel > 100)
        moistureLevel = 100;

      // If the interval is less than 2 seconds then don't turn the sensor off
      if (readingInterval > delayAfterTurningSensorOn)
      {
        sensorOff();
      }
    }
  }
}

int calculateMoistureLevel(int reading)
{
  return map(reading, dryReading, wetReading, 0, 100);
}

/* Serial Output */
void serialPrintData()
{
  if (lastSerialOutputTime + serialOutputInterval < millis()
      || lastSerialOutputTime == 0)
  {
    if (serialMode == SERIAL_MODE_CSV)
    {
      Serial.print("D;"); // This prefix indicates that the line contains data.
      Serial.print("Raw:");
      Serial.print(moistureLevelRaw);
      Serial.print(";");
      Serial.print("Calibrated:");
      Serial.print(moistureLevel);
      Serial.print(";");
      Serial.print("Threshold:");
      Serial.print(threshold);
      Serial.print(";");
      Serial.print("WaterNeeded:");
      Serial.print(moistureLevel < threshold);
      Serial.print(";");
      Serial.print("PumpOn:");
      Serial.print(pumpIsOn);
      Serial.print(";");
      Serial.print("SecondsSincePumpOn:");
      Serial.print((millis() - lastPumpFinishTime) / 1000);
      Serial.print(";");
      Serial.print("Dry:");
      Serial.print(dryReading);
      Serial.print(";");
      Serial.print("Wet:");
      Serial.print(wetReading);
      Serial.print(";");
      Serial.println();
    }
    else
    {
      Serial.print("raw=");
      Serial.print(moistureLevelRaw);
      Serial.print("&");
      Serial.print("calibrated=");
      Serial.print(moistureLevel);
      Serial.print("&");
      Serial.print("threshold=");
      Serial.print(threshold);
      Serial.print("&");
      Serial.print("waterNeeded=");
      Serial.print(moistureLevel < threshold);
      Serial.print("&");
      Serial.print("pumpOn=");
      Serial.print(pumpIsOn);
      Serial.print("&");
      Serial.print("secondsSincePumpOn=");
      Serial.print((millis() - lastPumpFinishTime) / 1000);
      Serial.print("&");
      Serial.print("dry=");
      Serial.print(dryReading);
      Serial.print("&");
      Serial.print("wet=");
      Serial.print(wetReading);
      Serial.println();
    }

    if (isDebug)
    {
      Serial.print("Last pump start time:");
      Serial.println(pumpStartTime);
      Serial.print("Last pump finish time:");
      Serial.println(lastPumpFinishTime);
    }

    lastSerialOutputTime = millis();
  }
}

/* Display */
void displayReading()
{
  if (millis() > lastDisplayTime + displayRefreshInterval)
  {
    lcd.clear();
    lcd.setCursor(0, 0);

    if (!isDebug)
    {
      displayFriendlyReading();
    }
    else
    {
      displayDebugReading();
    }

    lastDisplayTime = millis();
  }
}

void displayFriendlyReading()
{
  lcd.print("Moisture: ");
  lcd.print(moistureLevel);
  lcd.print("%");
  // Enable the following code to show the raw value
  //lcd.print(" (");
  //lcd.print(moistureLevelRaw);
  //lcd.print(")");

  lcd.setCursor(0, 1);
  if (pumpIsOn)
  {
    lcd.print("Pumping");
  }
  else if (calibrationMode == CALIBRATION_MODE_OFF)
  {
    lcd.print("Trigger at: ");
    lcd.print(threshold);
    lcd.print("%   ");
    // Enable the following code to display the calibrated range
    //lcd.print(dryReading);
    //lcd.print("/");
    //lcd.print(wetReading);
  }
  else if (calibrationMode == CALIBRATION_MODE_DRY)
  {
    lcd.print("Dry: ");
    lcd.print(dryReading);
    lcd.print("  Auto: ");
    lcd.print(moistureLevelRaw);
  }
  else if (calibrationMode == CALIBRATION_MODE_WET)
  {
    lcd.print("Wet:");
    lcd.print(wetReading);
    lcd.print("  Auto: ");
    lcd.print(moistureLevelRaw);
  }
}

void displayDebugReading()
{
  lcd.print(moistureLevel);
  lcd.print("% (");
  lcd.print(moistureLevelRaw);
  lcd.print(")");
  lcd.print("  T");
  lcd.print(threshold);

  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.print(dryReading);

  if (calibrationMode == CALIBRATION_MODE_DRY)
  {
    lcd.print("(c)");
  }

  lcd.print(" W:");
  lcd.print(wetReading);

  if (calibrationMode == CALIBRATION_MODE_WET)
  {
    lcd.print("(c)");
  }
}

void refreshDisplay() {
  // Reset the last display time to force a refresh
  lastDisplayTime = 0;

  displayReading();
}


void toggleScreenBacklight()
{
  screenBacklightIsOn = !screenBacklightIsOn;

  if (isDebug)
    Serial.println("Toggle screen backlight");

  if (screenBacklightIsOn)
    lcd.backlight();
  else
    lcd.noBacklight();
}

/* Irrigation */
void irrigateIfNeeded()
{
  if (pumpStatus == PUMP_STATUS_AUTO)
  {
    bool readingHasBeenTaken = lastReadingTime > 0;
    bool pumpBurstFinished = pumpStartTime + pumpBurstDuration < millis();
    bool waterIsNeeded = moistureLevel <= threshold && readingHasBeenTaken;
    bool pumpIsReady = lastPumpFinishTime + pumpWaitOffDuration < millis() || lastPumpFinishTime == 0;

    if (pumpIsOn)
    {
      if (pumpBurstFinished)
      {
        if (isDebug)
          Serial.println("Pump burst finished");
        pumpOff();
      }
    }
    else if (waterIsNeeded && pumpIsReady)
    {
      if (isDebug)
        Serial.println("Pump is turning on");
      pumpOn();
    }
  }
}

void pumpOn()
{
  digitalWrite(pumpPin, HIGH);
  pumpIsOn = true;

  pumpStartTime = millis();
}

void pumpOff()
{
  digitalWrite(pumpPin, LOW);
  pumpIsOn = false;

  lastPumpFinishTime = millis();
}

/* Sensor */
void sensorOn()
{
  if (isDebug)
    Serial.println("Turning sensor on");

  digitalWrite(sensorPowerPin, HIGH);

  lastSensorOnTime = millis();

  delay(delayAfterTurningSensorOn);

  sensorIsOn = true;
}

void sensorOff()
{
  if (isDebug)
    Serial.println("Turning sensor off");

  digitalWrite(sensorPowerPin, LOW);

  sensorIsOn = false;
}

/* Calibration */
void initiateDryCalibration()
{
  if (isDebug)
    Serial.println("Initiating dry calibration");

  calibrationMode = CALIBRATION_MODE_DRY;
  calibrationTriggerTime = millis();

  refreshDisplay();
}

void initiateWetCalibration()
{
  if (isDebug)
    Serial.println("Initiating wet calibration");

  calibrationMode = CALIBRATION_MODE_WET;
  calibrationTriggerTime = millis();

  refreshDisplay();
}

void confirmCalibrateDry()
{
  lastReadingTime = 0;
  takeReading();
  setDryReading(moistureLevelRaw);

  // Calibration mode is completed. Go back to normal
  calibrationMode = CALIBRATION_MODE_OFF;

  refreshDisplay();
}

void confirmCalibrateWet()
{
  lastReadingTime = 0;
  takeReading();
  setWetReading(moistureLevelRaw);

  // Calibration mode is completed. Go back to normal
  calibrationMode = CALIBRATION_MODE_OFF;

  refreshDisplay();
}

void checkCalibrationTimeout()
{
  if (calibrationMode != CALIBRATION_MODE_OFF
      && calibrationTriggerTime + calibrationInterval < millis())
  {
    if (calibrationMode == CALIBRATION_MODE_DRY)
      setDryReading(dryReading);

    if (calibrationMode == CALIBRATION_MODE_WET)
      setWetReading(wetReading);

    cancelCalibration();
    Serial.println("Calibration timed out");
  }
}

void cancelCalibration()
{
  calibrationMode = CALIBRATION_MODE_OFF;

  refreshDisplay();
}

void setDryReading(int reading)
{
  dryReading = reading;

  if (isDebug)
  {
    Serial.print("Setting dry reading: ");
    Serial.println(reading);
  }

  EEPROM.write(dryReadingAddress, reading / 4); // Must divide by 4 to make it fit in eeprom
}

void setWetReading(int reading)
{
  wetReading = reading;

  if (isDebug)
  {
    Serial.print("Setting wet reading: ");
    Serial.println(reading);
  }

  EEPROM.write(wetReadingAddress, reading / 4); // Must divide by 4 to make it fit in eeprom
}

int getDryReading()
{
  int value = EEPROM.read(dryReadingAddress);

  if (value == 255)
    return dryReading;
  else
    return value * 4; // Must multiply by 4 to get the original value
}

int getWetReading()
{
  int value = EEPROM.read(wetReadingAddress);

  if (value == 255)
    return wetReading;
  else
    return value * 4; // Must multiply by 4 to get the original value
}

