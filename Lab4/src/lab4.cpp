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

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t speed;

/* Declarations */
float seconds = 0.01;  // How many seconds between each timer interrupt
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
  delay(1000);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  for(int i = 6; i<ptrSensorBar->kNumOfSensors; i++) {
    pinMode(i, INPUT_PULLUP);
  }
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

  ptrController->tot_integral = 0;
  ptrController->pGain = 2;
  ptrController->iGain = 0.1;
  ptrController->pGainAngle = 1;
  ptrController->iGainAngle = 1;


  rightWheel.dcMotor = AFMS.getMotor(rightWheel.MOTOR_NUMBER);
  leftWheel.dcMotor = AFMS.getMotor(leftWheel.MOTOR_NUMBER);
  AFMS.begin();
  Serial.begin(9600);
  
  rightWheel.dcMotor->setSpeed(0);
  leftWheel.dcMotor->setSpeed(0);
  rightWheel.dcMotor->run(FORWARD);
  leftWheel.dcMotor->run(FORWARD);
  

   turnLeft(100, ptrRightWheel, ptrLeftWheel);
  sensCalib(ptrRightWheel, ptrLeftWheel, ptrSensorBar);
  turnRight(100, ptrRightWheel, ptrLeftWheel);
  sensCalib(ptrRightWheel, ptrLeftWheel, ptrSensorBar);
  turnLeft(0, ptrRightWheel, ptrLeftWheel); 
   ptrRightWheel->speed = 0;
  ptrLeftWheel->speed = 0;
  goStraight(ptrRightWheel, ptrLeftWheel);
  runMotors(ptrRightWheel, ptrLeftWheel); 
  delay(2000);
  Serial.println("Loop started");
}
float setVelocityR = 0;
float setVelocityL = 0;
int cntr = 0;
float sinCntr = 0;
double meanVelocity = 0;
int tic = 0;
int toc = 0;
float angle = 0;
bool reached = false;


void loop() {

  if (timed == true) {
    readAllSensorsDigital(ptrSensorBar);
    angleController(50, ptrSensorBar, ptrController);
    calculateVelocity(ptrRightWheel, ptrLeftWheel, seconds);
    
    if(ptrSensorBar->angleSetpoint < 0) {
      setVelocityL = (ptrSensorBar->angleSetpoint*150/(45*255)) + 0.5;
      setVelocityR = 0.5;
      Serial.print("Turning right: ");
      piController(setVelocityR, ptrController, ptrRightWheel);
      piController(setVelocityL, ptrController, ptrLeftWheel);
      Serial.println(ptrLeftWheel->speed);

    } else if(ptrSensorBar->angleSetpoint > 0) {
      ptrSensorBar->angleSetpoint = 0-ptrSensorBar->angleSetpoint;
      setVelocityR = (ptrSensorBar->angleSetpoint*150 / (45*255)) + 0.5;
      setVelocityL = 0.5;
      Serial.print("Turning left: ");
      piController(setVelocityR, ptrController, ptrRightWheel);
      piController(setVelocityL, ptrController, ptrLeftWheel);
      Serial.println(ptrRightWheel->speed);
    } else {
      setVelocityR = 0.5;
      setVelocityL = 0.5;
      piController(setVelocityR, ptrController, ptrRightWheel);
      piController(setVelocityL, ptrController, ptrLeftWheel);
    }
    rightWheel.dcMotor->setSpeed(ptrRightWheel->speed);
    leftWheel.dcMotor->setSpeed(ptrLeftWheel->speed);

   
  }

/* Position control */
//readAllSensorsDigital(ptrSensorBar);
/* for(int i = 0; i < ptrSensorBar->kNumOfSensors; i++) {
  if(ptrSensorBar->lineDetected[i] == true) {
    Serial.print("Line detected on sensor:    ");
    Serial.println(i);
    delay(500);
  } */

/*   else
  {
    Serial.println("No line");
  } */
  




/* Velocity control */
  /*   if (Serial.available() > 0) {
      ptrController->pGain = Serial.parseFloat();
      ptrController->iGain = Serial.parseFloat();
      setVelocity = 0;
      rightWheel.dcMotor->setSpeed(0);
      leftWheel.dcMotor->setSpeed(0);
      cntr = 0;
  }
    calculateVelocity(ptrRightWheel, ptrLeftWheel, seconds);
    piController(setVelocity, ptrController, ptrRightWheel, ptrLeftWheel);
    rightWheel.dcMotor->setSpeed(ptrRightWheel->speed);
    leftWheel.dcMotor->setSpeed(ptrLeftWheel->speed);
    speed.number = ptrRightWheel->velocity;
    if (timed == true) {
      for (int i = 0; i<4; i++) {
        meanVelocity = meanVelocity + ptrRightWheel->velocityArr[i];
      }
      meanVelocity = meanVelocity / 5;
      Serial.println(setVelocity*20);
      Serial.print(',');
      //Serial.println(ptrRightWheel->velocity*20);
      Serial.println(meanVelocity);
      timed = false;
      meanVelocity = 0;
    }
    cntr++;
    if (cntr > 100) {
      sinCntr += 0.1;
      setVelocity = 0.3*sin(sinCntr)+0.3;
      cntr = 0;
    } */
}
