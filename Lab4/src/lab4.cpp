/*
 *   Copyright (c) 2020 
 *   All rights reserved.
 */
#include <Wire.h>
#include "lab4.h"
#include <DueTimer.h>

/* Settings */
const uint8_t RIGHT_MOTOR_PORT = 4;
const uint8_t LEFT_MOTOR_PORT = 1;

/* Declarations */
float seconds = .05;  // How many seconds between each timer interrupt
double uTimerLength = 1000000*seconds;  // Recalculate to micro seconds
bool timed = false;  // Indicator that the timer interrupt has been triggered

// Create wheels, pointers and some other things
Wheel rightWheel = {0, 0, 0, RIGHT_MOTOR_PORT, 0, 0, FORWARD};
Wheel leftWheel  = {0, 0, 0, LEFT_MOTOR_PORT, 0, 0, BACKWARD};
Controller controller;
Wheel *ptrRightWheel, *ptrLeftWheel;
Controller *ptrController;
SensorBar sensorBar, *ptrSensorBar;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

/* Interrupt handlers */
void intrR() {
  rightWheel.counter++;
}

void intrL() {
  leftWheel.counter++;
}

void timerInterrupt() {
    rightWheel.timedPulses = rightWheel.counter - rightWheel.prevCounter;
    leftWheel.timedPulses = leftWheel.counter - leftWheel.prevCounter;
    rightWheel.prevCounter = rightWheel.counter;
    leftWheel.prevCounter = leftWheel.counter;
    timed = true;
}

/********************* SETUP *********************/
void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(8, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), intrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), intrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), intrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), intrL, CHANGE);
  Timer3.attachInterrupt(timerInterrupt);
  Timer3.start(uTimerLength);

  ptrRightWheel = &rightWheel;
  ptrLeftWheel = &leftWheel;
  ptrSensorBar = &sensorBar;
  ptrController = &controller;

  ptrController->tot_integral=0;


  rightWheel.dcMotor = AFMS.getMotor(rightWheel.MOTOR_NUMBER);
  leftWheel.dcMotor = AFMS.getMotor(leftWheel.MOTOR_NUMBER);
  AFMS.begin();
  rightWheel.dcMotor->run(FORWARD);
  leftWheel.dcMotor->run(FORWARD);
  rightWheel.dcMotor->setSpeed(0);
  leftWheel.dcMotor->setSpeed(0);
  Serial.begin(115200);

  turnLeft(100,ptrRightWheel, ptrLeftWheel);
  sensCalib(ptrRightWheel, ptrLeftWheel, ptrSensorBar);
  turnRight(100, ptrRightWheel, ptrLeftWheel);
  sensCalib(ptrRightWheel, ptrLeftWheel, ptrSensorBar);
  turnLeft(0, ptrRightWheel, ptrLeftWheel);
  ptrRightWheel->speed = 0;
  ptrLeftWheel->speed = 0;
  goStraight(ptrRightWheel, ptrLeftWheel);
  runMotors(ptrRightWheel, ptrLeftWheel);
}
float setVelocity = 0;
int cntr = 0;
void loop() {

  calculateVelocity(ptrRightWheel, ptrLeftWheel, seconds);
  //pController(setVelocity, 2, ptrRightWheel, ptrLeftWheel);
  piController(setVelocity, 2, 0.01, ptrController, ptrRightWheel, ptrLeftWheel);
  rightWheel.dcMotor->setSpeed(ptrRightWheel->speed);
  leftWheel.dcMotor->setSpeed(ptrLeftWheel->speed);
  //runMotors(ptrRightWheel, ptrLeftWheel);
  if (timed == true) {
    Serial.println(ptrRightWheel->velocity);
    timed = false;
  }
  cntr++;
  if (cntr > 1000) {
    setVelocity = 0.5;
  } 
}
