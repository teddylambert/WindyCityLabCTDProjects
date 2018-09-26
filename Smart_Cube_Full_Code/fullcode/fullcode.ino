#include "Adafruit_BNO055.h"
#include "LiquidCrystal.h"
#include "Adafruit_BMP180.h"
#include "blynk.h"

#define BLYNK_PRINT Serial

char auth[] = "40c3be5756094560a6e993026e3c924e";

bool alarmOn = false;
const int snoozeAmount = 300;
bool snoozing;
bool alarm1Enabled;
bool alarm1Active;
bool alarm2Active;
bool alarm2Enabled;

bool displayOn = true;

enum clockState {
  clockStateNormalOperation,
  clockStateSetTime,
  clockStateSetAlarm1,
  clockStateSetAlarm2,
  clockStateSetDate,
  clockStateWifi,
  clockStateCount
};
clockState setMode;

const int hourPin = 4;
const int minutePin = 5;
const int setPin = 6;

time_t alarm1 = 1466575200 + 18000;
time_t alarm2 = 1466578800 + 18000; //for timezone
time_t alarm1SnoozeTime;
time_t alarm2SnoozeTime;

int currentRotate = 0;
int setRotate;
int downRotate = 360;

bool hourButton = false;
bool minuteButton = false;
bool setButton = false;

bool hbuttonState;
bool mbuttonState;
bool sbuttonState;

bool previousHstate;
bool previousMstate;
bool previousSstate;

volatile bool goodTime = true;
volatile bool getTemp = true;
volatile bool goodDate = true;

int DEBOUNCE_TIMEOUT = 100;
int DEBOUNCE_TIME = 10;

Adafruit_BNO055 bno = Adafruit_BNO055();
LiquidCrystal lcd(0);

Adafruit_BMP085 bmp;

Timer timer(1000, getTime);
Timer timer2(10000, Temp);
Timer timer3(1000, getDate);

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Time.zone(-5);
  lcd.begin(16, 2);
  if (!bno.begin())
  {
    Serial.print(
      "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  hbuttonState = digitalRead(hourPin);
  mbuttonState = digitalRead(minutePin);
  sbuttonState = digitalRead(setPin);
  previousHstate = hbuttonState;
  previousMstate = mbuttonState;
  previousSstate = sbuttonState;
  lcd.clear();
  Blynk.begin(auth);
  InitializeApplication();
  timer.start();
  timer2.start();
  timer3.start();
  InitializeBMP085();
  int8_t temp = bno.getTemp();
  bno.setExtCrystalUse(true);
  alarm1Enabled = false;
  alarm2Enabled = false;
  synchronizeSnoozeTimesToAlarmTimes();
  shieldReset();
  setMode = clockStateNormalOperation;
}

BLYNK_WRITE(V1)
{
  if (param.asInt() == 1)
  {
    lcd.setBacklight(LOW);
    lcd.noDisplay();
  }
}

BLYNK_WRITE(V2)
{
  if (param.asInt() == 1)
  {
    lcd.setBacklight(HIGH);
    lcd.display();
  }
}

BLYNK_WRITE(V7)
{
  if (param.asInt() == 1)
  {
    Serial1.write('x');
  }
}

BLYNK_WRITE(V5)
{
  if (param.asInt() == 1)
  {
    Serial1.write('y');
  }
}

BLYNK_WRITE(V6)
{
  if (param.asInt() == 1)
  {
    Serial1.write('z');
  }
}

void loop() {
  debounce();
  Blynk.run();
  if (setButton == true && setMode == clockStateNormalOperation)
  {
    setButton = false;
    lcd.clear();
    setMode = clockStateSetTime;
    Serial.println("set time");
  }
  if (setMode == clockStateNormalOperation)
  {
    getOrientation();
    volumeControl();
    getG();
    updateDisplayNormalOperation();
    monitorAlarmButtons();
    checkAlarms();
    if (alarm1Active == true || alarm2Active == true)
    {
      monitorSnooze();
    }
  }
  if (setMode != clockStateNormalOperation)
  {
    blinkDisplay();
    updateDisplaySetMode();
    monitorHourMinButtons();
  }
  if (setButton == true && setMode == clockStateSetTime)
  {
    setButton = false;
    minuteButton = false;
    hourButton = false;
    lcd.clear();
    setMode = clockState(int(setMode) + 1);
    Serial.println("set alarm 1");
  }
  if (setButton == true && setMode == clockStateSetAlarm1)
  {
    setButton = false;
    minuteButton = false;
    hourButton = false;
    lcd.clear();
    setMode = clockState(int(setMode) + 1);
    Serial.println("set alarm 2");
  }
  if (setButton == true && setMode == clockStateSetAlarm2 || setButton == true
    && setMode == clockStateSetDate || setButton == true && setMode == clockStateWifi)
  {
    setButton = false;
    minuteButton = false;
    hourButton = false;
    lcd.clear();
    setMode = clockState(int(setMode) + 1);
  }
  if (setMode == clockStateCount)
  {
    setButton = false;
    minuteButton = false;
    hourButton = false;
    lcd.clear();
    setMode = clockStateNormalOperation;
    lcd.display();
    goodDate = true;
    getTemp = true;
    Serial.println("normal op");
  }
}

void shieldReset()
{
  Serial1.write('r');
}

void InitializeBMP085()
{
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
}

void PublishBMP085Info()
{
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  Serial.print("Temperature = ");
  Serial.print((bmp.readTemperature() * 1.8 + 32));
  Serial.println(" *F");
  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(101500));
  Serial.println(" meters");
  char szEventInfo[64];
  sprintf(szEventInfo, "Temperature=%.2f °C, Pressure=%.2f hPa", bmp.readTemperature(), bmp.readPressure() / 100.0);

  Spark.publish("bmpo85info", szEventInfo);
}

void InitializeApplication()
{
  Serial.begin(9600);
  pinMode(D7, OUTPUT);
}

void getG()
{
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print("X: ");
  Serial.println(accel.x());
  Serial.print(" Y: ");
  Serial.println(accel.y());
  Serial.print(" Z: ");
  Serial.println(accel.z());
  Blynk.virtualWrite(V0, accel.z());

}

void getOrientation()
{
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("X:");
  Serial.println(event.orientation.x);
  Serial.print("Y:");
  Serial.println(event.orientation.y);
  Serial.print("Z:");
  Serial.println(event.orientation.z);
  Blynk.virtualWrite(V8, event.orientation.x);
}

void getTime()
{
  goodTime = true;
}
void Temp()
{
  getTemp = true;
}
void getDate()
{
  goodDate = true;
}

void volumeControl()
{
  sensors_event_t event;
  bno.getEvent(&event);
  setRotate = (event.orientation.x);
  if ((setRotate - currentRotate > 10) && (setRotate < 175))
  {
    currentRotate = (event.orientation.x);
    Serial1.write('u');
    Serial.println("sent");
  }
  if ((event.orientation.x == 0))
  {
    currentRotate = 0;
    downRotate = 360;
    //Serial.println("zero");
  }
  if ((downRotate - setRotate > 10) && (setRotate > 185))
  {
    downRotate = (event.orientation.x);
    Serial1.write('d');
    Serial.println("down");
  }
}

void blinkDisplay()
{
  static long lastUpdate;
  if (millis() - lastUpdate > 500)
  {
    lastUpdate = millis();
    displayOn = !displayOn;
    if (displayOn == true)
    {
      lcd.noDisplay();
    }
    else
    {
      lcd.display();
    }
  }
}

void updateDisplaySetMode()
{
  switch (setMode) {
    case clockStateSetTime:
      {
        {
          lcd.setCursor(0, 0);
          lcd.print(Time.hourFormat12());
          lcd.print(":");
          if (Time.minute() < 10)
          {
            lcd.print("0");
            lcd.print(Time.minute());
          }
          else
          {
            lcd.print(Time.minute());
          }
          if (Time.isAM() == true)
          {
            lcd.print("A");
          }
          else
          {
            lcd.print("P");
          }
        }
      } break;
    case clockStateSetAlarm1:
      {
        lcd.setCursor(0, 1);
        lcd.print(Time.hourFormat12(alarm1));
        lcd.print(":");
        if (Time.minute(alarm1) < 10)
        {
          lcd.print("0");
          lcd.print(Time.minute(alarm1));
        }
        else
        {
          lcd.print(Time.minute(alarm1));
        }
        if (Time.isAM(alarm1) == true)
        {
          lcd.print("A");
        }
        else
        {
          lcd.print("P");
        }
      } break;
    case clockStateSetAlarm2:
      {
        lcd.setCursor(8, 1);
        lcd.print(Time.hourFormat12(alarm2));
        lcd.print(":");
        if (Time.minute(alarm2) < 10)
        {
          lcd.print("0");
          lcd.print(Time.minute(alarm2));
        }
        else
        {
          lcd.print(Time.minute(alarm2));
        }
        if (Time.isAM(alarm2) == true)
        {
          lcd.print("A");
        }
        else
        {
          lcd.print("P");
        }
      } break;
    case clockStateSetDate:
      {
        lcd.setCursor(8, 0);
        if (Time.month() < 10)
        {
          lcd.print("0");
          lcd.print(Time.month());
        }
        else
        {
          lcd.print(Time.month());
        }
        lcd.print("/");
        if (Time.day() < 10)
        {
          lcd.print("0");
          lcd.print(Time.day());
        }
        else
        {
        lcd.print(Time.day());
        }
        lcd.print("/");
        lcd.println(Time.year() - 2000);
      } break;
    case clockStateWifi:
    {
      lcd.setCursor(0,0);
      if (WiFi.ready() == false)
      {
        lcd.print("Disconnected");
      }
      if (WiFi.ready() == true)
      {
        lcd.print("connected");
      }
      if (WiFi.connecting() == true)
      {
        lcd.print("connecting...");
      }
    }
  }
}

void monitorHourMinButtons()
{
  if (hourButton == true && setMode == clockStateSetTime) //would a while loop work here?
  {
    Serial.println("hour+");
    lcd.clear();
    Time.setTime(Time.now() + 3600);
    hourButton = false;
  }
  if (hourButton == true && setMode == clockStateSetAlarm1)
  {
    Serial.println("alarm1hour+");
    lcd.clear();
    alarm1 = alarm1 + 3600;
    hourButton = false;
  }
  if (hourButton == true && setMode == clockStateSetAlarm2)
  {
    Serial.println("alarm2hour+");
    lcd.clear();
    alarm2 = alarm2 + 3600;
    hourButton = false;
  }
  if (hourButton == true && setMode == clockStateSetDate)
  {
    lcd.clear();
    Time.setTime(Time.now() - 86400);
    hourButton = false;
  }
  if (hourButton == true && setMode == clockStateWifi)
  {
    lcd.clear();
    WiFi.disconnect();
    hourButton = false;
  }
  if (minuteButton == true && setMode == clockStateSetTime)
  {
    Serial.println("minute+");
    lcd.clear();
    Time.setTime(Time.now() + 60);
    minuteButton = false;
  }
  if (minuteButton == true && setMode == clockStateSetAlarm1)
  {
    Serial.println("alarm1minute+");
    lcd.clear();
    alarm1 = alarm1 + 60;
    minuteButton = false;
  }
  if (minuteButton == true && setMode == clockStateSetAlarm2)
  {
    Serial.println("alarm2minute+");
    lcd.clear();
    alarm2 = alarm2 + 60;
    minuteButton = false;
  }
  if (minuteButton == true && setMode == clockStateSetDate)
  {
    lcd.clear();
    Time.setTime(Time.now() + 86400);
    minuteButton = false;
  }
  if (minuteButton == true && setMode == clockStateWifi)
  {
    lcd.clear();
    Particle.connect();
    minuteButton = false;
  }
  synchronizeSnoozeTimesToAlarmTimes();
}
void monitorAlarmButtons()
{
  if (hourButton == true)
  {
    hourButton = false;
    if (alarm1Active == true)
    {
      alarm1Active = false;
      synchronizeSnoozeTimesToAlarmTimes();
      stopAlarm();
    }
    else
    {
      alarm1Enabled = !alarm1Enabled;
      Serial.println("alarm 1 toggle");
    }
  }
  if (minuteButton == true && setMode == clockStateNormalOperation) {
    minuteButton = false;
    if (alarm2Active == true)
    {
      alarm2Active = false;
      synchronizeSnoozeTimesToAlarmTimes();
      stopAlarm();
    }
    else
    {
      alarm2Enabled = !alarm2Enabled;
      Serial.println("alarm 2 toggle");
    }
  }
}
void synchronizeSnoozeTimesToAlarmTimes()
{
  alarm1SnoozeTime = alarm1;
  alarm2SnoozeTime = alarm2;
}
void monitorSnooze()
{
  sensors_event_t event;
  bno.getEvent(&event);
  if (event.orientation.y > 20)
  {
    snoozeHit();
    Serial.println("snooze has hit");
  }
}

void startAlarm1()
{
  Serial.println("starting alarm");
  Serial1.write('x');
}

void startAlarm2()
{
  Serial.println("alarm2");
  Serial1.write('y');
}

void stopAlarm()
{
  Serial1.write('z');
}


void snoozeHit()
{
  if (alarm1Active == true)
  {
    alarm1Active = false;
    alarm1SnoozeTime = Time.now() + snoozeAmount;
  }
  if (alarm2Active == true)
  {
    alarm2Active = false;
    alarm2SnoozeTime = Time.now() + snoozeAmount;
  }
  stopAlarm();
}

void writeAlarmStatus()
{
  lcd.setCursor(0, 1);
  if (alarm1Enabled)
  {
    lcd.print("1:Y"); //change this
  }
  else
  {
    lcd.print("1:N");
  }
  lcd.setCursor(4, 1);
  if (alarm2Enabled)
  {
    lcd.print("2:Y"); //change this
  }
  else
  {
    lcd.print("2:N");
  }
}

void updateDisplayNormalOperation() {
  if (goodTime == true) {
    goodTime = false;
    lcd.setCursor(0, 0);
    lcd.print(Time.hourFormat12());
    lcd.print(":");
    if (Time.minute() < 10)
    {
      lcd.print("0");
      lcd.print(Time.minute());
    }
    else
    {
      lcd.print(Time.minute());
    }
    if (Time.isAM() == true)
    {
      lcd.print("A");
    }
    else
    {
      lcd.print("P");
    }
  }
  if (getTemp == true)
  {
    getTemp = false;
    lcd.setCursor(11, 1);
    lcd.print((bmp.readTemperature() * 1.8 + 32));
    Blynk.virtualWrite(V3, bmp.readTemperature() * 1.8 + 32);

  }
  writeAlarmStatus();
  if (goodDate == true)
  {
    goodDate = false;
    lcd.setCursor(8, 0);
    if (Time.month() < 10)
    {
      lcd.print("0");
      lcd.print(Time.month());
    }
    else
    {
      lcd.print(Time.month());
    }
    lcd.print("/");
    if (Time.day() < 10)
    {
      lcd.print("0");
      lcd.print(Time.day());
    }
    else
    {
    lcd.print(Time.day());
    }
    lcd.print("/");
    lcd.println(Time.year() - 2000);
  }
}

void checkAlarms()
{
  if ((Time.hourFormat12() == Time.hourFormat12(alarm1) && Time.minute() == Time.minute(alarm1)
       && Time.second() == Time.second(alarm1) && alarm1Enabled == true && alarm1Active == false) || (Time.hourFormat12() ==
           Time.hourFormat12(alarm1SnoozeTime) && Time.minute() == Time.minute(alarm1SnoozeTime)
           && Time.second() == Time.second(alarm1SnoozeTime) && alarm1Enabled == true && alarm1Active == false))
  {
    alarm1Active = true;
    startAlarm1();
  }
  if ((Time.hourFormat12() == Time.hourFormat12(alarm2) && Time.minute() == Time.minute(alarm2)
       && Time.second() == Time.second(alarm2) && alarm2Enabled == true && alarm2Active == false) || (Time.hourFormat12() ==
           Time.hourFormat12(alarm2SnoozeTime) && Time.minute() == Time.minute(alarm2SnoozeTime)
           && Time.second() == Time.second(alarm2SnoozeTime) && alarm2Enabled == true && alarm2Active == false))
  {
    alarm2Active = true;
    startAlarm2();
  }
}




/*bool ModeBtnPressed() {
    if(millis() > 5000) {
        if(BUTTON_GetDebouncedTime(BUTTON1) >= 50) {
            BUTTON_ResetDebouncedState(BUTTON1);
            return 1;
        }
    }
    return 0;
  }
*/
void debounce()
{
  hbuttonState = digitalRead(hourPin);
  if (hbuttonState != previousHstate)
  {
    bool value = buttonDebounce(hbuttonState, 4);
    if ((value == 1) && (hbuttonState == LOW))
    {
      hourButton = true;
    }
  }
  previousHstate = hbuttonState;

  mbuttonState = digitalRead(minutePin);
  if (mbuttonState != previousMstate)
  {
    bool value = buttonDebounce(mbuttonState, 5);
    if ((value == 1) && (mbuttonState == LOW))
    {
      minuteButton = true;
    }
  }
  previousMstate = mbuttonState;

  sbuttonState = digitalRead(setPin);
  if (sbuttonState != previousSstate)
  {
    bool value = buttonDebounce(sbuttonState, 6);
    if ((value == 1) && (sbuttonState == LOW))
    {
      setButton = true;
    }
  }
  previousSstate = sbuttonState;
}


int buttonDebounce(int state, int pinNumber)
{
  unsigned long start = millis();
  int debounce_start = start;

  while (millis() - start < DEBOUNCE_TIMEOUT)
  {
    int value = digitalRead(pinNumber);
    if (value == state)
    {
      if (millis() - debounce_start >= DEBOUNCE_TIME)
        return 1;
    }
    else
      debounce_start = millis();
  }
  return 0;
}
