#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include"utility/Adafruit_MS_PWMServoDriver.h"
#include "laba.h"
#include <DueTimer.h>

char incomingChar;
int incomingInt = 0;
int incomingInt2 = 0;
int lspeed = 0;
int rspeed = 0;
int dir = 1;
int dirL = 1;
int dirR = 1;
float radius = 0.02;
int revs = 0;
float omegaR = 6.28*revs;
float omegaL =  6.28*revs;

float velocityR = 0;
float velocityL = 0;
int rEnc = 3;
int lEnc = 4;
int rCntr = 0;
int lCntr = 0;
double seconds = 1;
double uTimerLength = 1000000*seconds;
double rPulsesPerTime = 0;
double lPulsesPerTime = 0;
int rState = 0;
int lState = 0;
int oldTime = 0;
int currentTime = 0;
int oldRCntr = 0;
int oldLCntr = 0;
int meters = 1;
int   newData = 0;
int pulses = (meters/0.04);

void intrR(){
  rCntr++;
  //rState = 1;
}
void intrL(){
  lCntr++;
  //rState = 1;
}

void timerInterrupt(){
    rPulsesPerTime = rCntr - oldRCntr;
    lPulsesPerTime = lCntr - oldLCntr;
    oldRCntr = rCntr;
    oldLCntr = lCntr;
    rState = 1;
    lState = 1;
}



// Create motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

void setup() {
  Serial.begin(115200);
  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(8, INPUT);
  //attachInterrupt(digitalPinToInterrupt(1), intrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), intrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), intrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), intrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), intrL, CHANGE);
  Timer3.attachInterrupt(timerInterrupt);
  Timer3.start(uTimerLength); 
  
  AFMS.begin();
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  delay(1000);

}

void loop() {
//
//  if((rState > 0 || lState > 0) && newData == 1){
//        omegaR = 2*3.14*rPulsesPerTime/(618*seconds);
//        velocityR = radius*omegaR;
//        omegaL = 2*3.14*lPulsesPerTime/(618*seconds);
//        velocityL = radius*omegaL;
//        lState  = 0;
//        rState = 0;
//        //Serial.print("Total r pulses: ");
//        //Serial.println(rCntr);
//        Serial.print("Velocity R: ");
//        Serial.println(velocityR);
//        Serial.print("Velocity L: ");
//        Serial.println(velocityL);
//        oldTime = millis();
//  }
//
//  
//  if (Serial.available()>0)
//  {
//    incomingInt = Serial.parseInt();
//    incomingInt2 = Serial.parseInt();   
//    // Determine direction
//    if (incomingInt < 0){
//      dirL = BACKWARD;
//     }
//     else{
//      dirL = FORWARD;
//     }
//    if (incomingInt2 < 0){
//      dirR = BACKWARD;
//     }
//     else{
//      dirR = FORWARD;
//     }
//      leftMotor->run(dirL);
//      rightMotor->run(dirR);
//     // Set speed
//    leftMotor->setSpeed(abs(incomingInt));
//    rightMotor->setSpeed(abs(incomingInt2));
//    newData = 1;
//  }

if(rCntr && lCntr <= 4918){
    Serial.println(rCntr);
    leftMotor->setSpeed(150);
    rightMotor->setSpeed(150);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
}
else{
leftMotor->setSpeed(0);
rightMotor->setSpeed(0);
}


  

}
