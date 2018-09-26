#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "types3.h"
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

enum motorState {
  off,
  upHigh,
  upLow,
  rightHigh,
  rightLow,
  leftHigh,
  leftLow,
  downHigh,
  downLow,
  button2,
  pedMode,
  polMode,
  count
};

#define powerPin 15
#define leftSensor 16
#define rightSensor 14
#define groundPin 7
#define redLEDs 13
#define blueLEDs 5
#define buzzer 3

#define lineReadLeft 860 //Use lab 1.5 to set these values
#define lineReadRight 800

int RedledState = LOW;
int BlueledState = LOW;
unsigned long previousMillis = 0;

const long interval = 333;
int toneUp = 0;

int sirenTone = 440;
int sirenType = 0;
int previousType = 0;
bool up = true;

bool policeMode = false;

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *leftFront = AFMS.getMotor(1);
Adafruit_DCMotor *leftBack = AFMS.getMotor(2);
Adafruit_DCMotor *rightFront = AFMS.getMotor(3);
Adafruit_DCMotor *rightBack = AFMS.getMotor(4);

bool motorControl = true;

void setup() {
  Serial.begin(9600);
  pinMode(powerPin, OUTPUT);
  pinMode(redLEDs, OUTPUT);
  pinMode(blueLEDs, OUTPUT);
  pinMode(groundPin, OUTPUT);
  digitalWrite(groundPin, LOW);
  digitalWrite(powerPin, HIGH);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  AFMS.begin();
  setUpMotors();
  ble.begin();
  ble.echo(false);
  ble.verbose(false);  // debug info is a little annoying after this point!
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println(F("In data mode"));
}

void loop()
{
  if (motorControl == true)
  {
    motorsRun();
    if (policeMode == true)
    {
      policeCar();
    }
    else
    {
      noTone(3);
      digitalWrite(redLEDs, LOW);
      digitalWrite(blueLEDs, LOW);
    }
  }
  else
  {
    stateMachine();
  }
}
void setUpMotors()
{
  leftFront->setSpeed(190); //Change speed accordingly
  //  leftFront->run(FORWARD);
  leftFront->run(RELEASE);

  rightFront->setSpeed(190);
  //  rightFront->run(FORWARD);
  rightFront->run(RELEASE);

  leftBack->setSpeed(190);
  //  leftBack->run(FORWARD);
  leftBack->run(RELEASE);

  rightBack->setSpeed(190);
  //  rightBack->run(FORWARD);
  rightBack->run(RELEASE);
}

void setWheels(int theOutputs)
{

  if (theOutputs & (1 << 1))
  {
    leftFront->run(FORWARD);
    leftBack->run(FORWARD);
  }
  else
  {
    leftFront->run(BACKWARD);
    leftBack->run(BACKWARD);
  }
  if (theOutputs & (1 << 0))
  {
    rightFront->run(FORWARD);
    rightBack->run(FORWARD);
  }
  else
  {
    rightFront->run(BACKWARD);
    rightBack->run(BACKWARD);
  }
}
void getInputs()
{
  if (analogRead(leftSensor) > lineReadLeft && analogRead(rightSensor) > lineReadRight)
  {
    currentInput = onLine;
  }
  if (analogRead(leftSensor) < lineReadLeft && analogRead(rightSensor) > lineReadRight)
  {
    currentInput = offLeft;
  }
  if (analogRead(leftSensor) > lineReadLeft && analogRead(rightSensor) < lineReadRight)
  {
    currentInput = offRight;
  }
  if (analogRead(leftSensor) < lineReadLeft && analogRead(rightSensor) < lineReadRight)
  {
    currentInput = lost;
  }
}

bool isHigh()
{
  while (!ble.available()) {}
  return (ble.read() == 49);
}

int remoteController()
{
  if (ble.available()) {
    if (ble.read() == '!')
    {
      while (!ble.available()) {}
      if (ble.read() == 'B')
      {
        while (!ble.available()) {}
        switch (ble.read())
        {
          case 50:
            {
              return button2;
              Serial.println("in state mode");
            } break;
          case 51:
            {
              return pedMode;
            } break;
          case 52:
            {
              return polMode;
            } break;
          case 53:
            {
              Serial.println("got 53!");
              if (isHigh()) return upHigh;
              else return upLow;
            } break;
          case 54:
            {
              Serial.println("got 54!");
              if (isHigh()) return downHigh;
              else return downLow;
            } break;
          case 55:
            {
              Serial.println("got 55");
              if (isHigh()) return leftHigh;
              else return leftLow;
            } break;
          case 56:
            {
              Serial.println("got 56");
              if (isHigh()) return rightHigh;
              else return rightLow;
            } break;
        }

      }
    }
  }

}
void motorsRun()
{
  switch (remoteController()) {
    case upHigh:
      {
        forwards();
      } break;
    case upLow:
      {
        stopMotors();
      } break;
    case leftHigh:
      {
        turnLeft();
      } break;
    case leftLow:
      {
        stopMotors();
      } break;
    case rightHigh:
      {
        turnRight();
      } break;
    case rightLow:
      {
        stopMotors();
      } break;
    case downHigh:
      {
        backwards();
      } break;
    case downLow:
      {
        stopMotors();
      } break;
    case button2:
      {
        stopMotors();
        motorControl = false;
      } break;
    case pedMode:
      {
        policeMode = false;
      } break;
    case polMode:
      {
        policeMode = true;
      }
  }
}

void forwards()
{
  leftFront->run(FORWARD);
  leftBack->run(FORWARD);
  rightFront->run(FORWARD);
  rightBack->run(FORWARD);
}

void backwards()
{
  leftFront->run(BACKWARD);
  leftBack->run(BACKWARD);
  rightFront->run(BACKWARD);
  rightBack->run(BACKWARD);
}

void turnLeft()
{
  leftFront->run(BACKWARD);
  leftBack->run(BACKWARD);
  rightFront->run(FORWARD);
  rightBack->run(FORWARD);
}

void turnRight()
{
  leftFront->run(FORWARD);
  leftBack->run(FORWARD);
  rightFront->run(BACKWARD);
  rightBack->run(BACKWARD);
}

void stopMotors()
{
  leftFront->run(RELEASE);
  leftBack->run(RELEASE);
  rightFront->run(RELEASE);
  rightBack->run(RELEASE);
}

void stateMachine()
{
  uint8_t currentOutputs = fsm[currentState].output;
  setWheels(currentOutputs);
  delay(fsm[currentState].checkTime);
  getInputs();
  currentState = fsm[currentState].nextState[currentInput];
  if (ble.available()) {
    if (ble.read() == '!')
    {
      while (!ble.available()) {}
      if (ble.read() == 'B')
      {
        while (!ble.available()) {}
        if (ble.read() == 49)
        {
          stopMotors();
          motorControl = true;
        }
      }
    }
  }
}


void policeCar()
{
  LEDs();
  siren();
}

void LEDs()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (RedledState == LOW) {
      RedledState = HIGH;
    }
    else
    {
      RedledState = LOW;
    }
    if (RedledState == LOW)
    {
      BlueledState = HIGH;
    }
    else
    {
      BlueledState = LOW;
    }
    digitalWrite(redLEDs, RedledState);
    digitalWrite(blueLEDs, BlueledState);


  }
}
void siren()
{
  if (up == true)
  {
    tone(3, sirenTone);
    sirenTone++;
    if (sirenTone == 1700)
    {
      up = false;
    }
  }
  if (up == false)
  {
    tone(3, sirenTone);
    sirenTone--;
    if (sirenTone == 440)
    {
      up = true;
      sirenType++;
      Serial.println(sirenType);
    }
  }
  if (sirenType - previousType >= 3)
  {
    toneUp++;
    if (toneUp < 1000)
    {
    tone(3, 250, 1000);
    }
    if (toneUp >=1000)
    {
    toneUp = 0;
    sirenType = previousType;
    sirenType++;
    sirenTone = 441;
    }
  }
}

