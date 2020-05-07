/*
 *   Copyright (c) 2020 
 *   All rights reserved.
 */
#include <Wire.h>
#include "lab4.h"
#include <DueTimer.h>

#define HC06 Serial1

/* Settings */
const uint8_t RIGHT_MOTOR_PORT = 4;
const uint8_t LEFT_MOTOR_PORT = 1;

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t speed;

/* Declarations */
float seconds = 0.005;  // How many seconds between each timer interrupt MIN: 0.004;
double uTimerLength = 1000000*seconds;  // Recalculate to micro seconds
bool timed = false;  // Indicator that the timer interrupt has been triggered

// Create wheels, pointers and some other things
Wheel rightWheel = {0, 0, 0, RIGHT_MOTOR_PORT, 0, 0, FORWARD};
Wheel leftWheel  = {0, 0, 0, LEFT_MOTOR_PORT, 0, 0, BACKWARD};
Controller controllerRight;
Controller controllerLeft;
Wheel *ptrRightWheel, *ptrLeftWheel;
Controller *ptrControllerRight;
Controller *ptrControllerLeft;
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
 // if(timed==false) {
    rightWheel.timedPulses = rightWheel.counter - rightWheel.prevCounter;
    leftWheel.timedPulses = leftWheel.counter - leftWheel.prevCounter;
    rightWheel.prevCounter = rightWheel.counter;
    leftWheel.prevCounter = leftWheel.counter;
    timed = true;
  //}
}

/********************* SETUP *********************/
void setup() {
  delay(1000);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  for(int i = sensorBar.sensorPins[0]; i<ptrSensorBar->kNumOfSensors; i++) {
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
  ptrControllerRight = &controllerRight;
  ptrControllerLeft = &controllerLeft;

  ptrControllerRight->tot_integral = 0;
  ptrControllerRight->pGain = 2;
  ptrControllerRight->iGain = 0.1;

  ptrControllerLeft->tot_integral = 0;
  ptrControllerLeft->pGain = 2;
  ptrControllerLeft->iGain = 0.1;


  

  rightWheel.dcMotor = AFMS.getMotor(rightWheel.MOTOR_NUMBER);
  leftWheel.dcMotor = AFMS.getMotor(leftWheel.MOTOR_NUMBER);
  AFMS.begin();
  Serial.begin(9600);
  HC06.begin(115200);
  
  rightWheel.dcMotor->setSpeed(0);
  leftWheel.dcMotor->setSpeed(0);
  rightWheel.dcMotor->run(FORWARD);
  leftWheel.dcMotor->run(FORWARD);
  

/*  turnLeft(100, ptrRightWheel, ptrLeftWheel);
  sensCalib(ptrRightWheel, ptrLeftWheel, ptrSensorBar);
  turnRight(100, ptrRightWheel, ptrLeftWheel);
  sensCalib(ptrRightWheel, ptrLeftWheel, ptrSensorBar);
  turnLeft(0, ptrRightWheel, ptrLeftWheel); 
   ptrRightWheel->speed = 0;
  ptrLeftWheel->speed = 0;
  goStraight(ptrRightWheel, ptrLeftWheel);
  runMotors(ptrRightWheel, ptrLeftWheel);   */
  delay(2000);
  Serial.println("Loop started");
}
float setVelocity = 0.2;
float setVelocityR = 0;
float setVelocityL = 0;
int cntr = 0;
float sinCntr = 0;
double meanVelocity = 0;
int tic = 0;
int toc = 0;
float angle = 0;
bool reached = false;
bool run = false;

float velocity = 0;

void loop() {

if (HC06.available() > 0) {
  rightWheel.dcMotor->setSpeed(0);
  ptrLeftWheel->dcMotor->setSpeed(0);
  ptrSensorBar->positionP = HC06.parseFloat();
  ptrSensorBar->positionI = HC06.parseFloat();
  ptrSensorBar->positionD = HC06.parseFloat();
  ptrSensorBar->errorInt = 0;
  ptrSensorBar->errorDot = 0;
  ptrSensorBar->error = 0;
  delay(1000);
  run = true;
}

if (timed == true && run == true){
  tic = micros();
  readAllSensorsDigital(ptrSensorBar);
  findGoalPosition(ptrSensorBar);
  calculateVelocity(ptrRightWheel, seconds); 
  calculateVelocity(ptrLeftWheel, seconds); 
  setVelocityR = 1+ptrSensorBar->velocitySetpoint;
  setVelocityL = 1-ptrSensorBar->velocitySetpoint;
  piController(0.3*setVelocityR, ptrControllerRight, ptrRightWheel);
  piController(0.3*setVelocityL, ptrControllerLeft, ptrLeftWheel);
  rightWheel.dcMotor->setSpeed(ptrRightWheel->speed);
  ptrLeftWheel->dcMotor->setSpeed(ptrLeftWheel->speed);
  //rightWheel.dcMotor->setSpeed(0);
  //ptrLeftWheel->dcMotor->setSpeed(0);
  //HC06.println(ptrSensorBar->goalY);
  
      for(int i = 0; i < 16; i++) {
    HC06.print(sensorBar.lineDetected[i]);
  }
  HC06.println("");
/*   HC06.print(ptrSensorBar->goalY*10);  
  HC06.print(",");
  HC06.println(0); */
  //HC06.println(ptrSensorBar->errorInt);
  toc = micros();
}
//Serial.println(toc-tic);
/*    for(int i = 0; i < 16; i++) {
    Serial.print(sensorBar.lineDetected[i]);
  }  
  Serial.print("    ");
  Serial.print("ThetaDot: ");
  Serial.print(ptrSensorBar->thetaDot);
  Serial.print("    Theta: ");
  Serial.println(ptrSensorBar->theta); */


  /* 
  rightWheel.dcMotor->setSpeed(0);
  ptrLeftWheel->dcMotor->setSpeed(0);
   */
/*   


  delay(100);   */
  



  
  
}
