#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include"utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

static const uint8_t sensorPins[] = {A0, A1, A2, A7, A8, A9, A10, A11};
int calibrationCounter = 0;
int tmp = 0;
int maxSensorValues[8];
int motorSpeed = 0;
int sensorValues[8];


void stopMotors(){
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
}

void goForward(int mtrSpeed){
  leftMotor->setSpeed(mtrSpeed);
  rightMotor->setSpeed(mtrSpeed); 
}


void setup() {
  delay(1000);
  // put your setup code here, to run once:
AFMS.begin();
leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(100);
  while(calibrationCounter < 20000){
  for(int i = 0; i<8; i++){
    tmp = analogRead(sensorPins[i]);
    if(tmp > maxSensorValues[i]){
      maxSensorValues[i] = tmp;
    }
  }
 calibrationCounter++;
  
  }
calibrationCounter = 0;
leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
    while(calibrationCounter < 20000){
  for(int i = 0; i<8; i++){
    tmp = analogRead(sensorPins[i]);
    if(tmp > maxSensorValues[i]){
      maxSensorValues[i] = tmp;
    }
  }
 calibrationCounter++;
  
  }

  

  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);

  while(analogRead(A7)<maxSensorValues[3]*0.7){
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  leftMotor->setSpeed(40);
  rightMotor->setSpeed(40);  
  }
  stopMotors();


}

void loop() {
  // put your main code here, to run repeatedly:
leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
for(int i = 0; i<8; i++){
    sensorValues[i] = analogRead(sensorPins[i]);
    delay(1);
}



if(sensorValues[3]>=maxSensorValues[3]*0.7){
  motorSpeed = 150;
  goForward(motorSpeed);
}

else if(sensorValues[2]>=maxSensorValues[2]*0.7){
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(motorSpeed);
}


else if(sensorValues[4]>=maxSensorValues[4]*0.7){
  rightMotor->setSpeed(0);
  leftMotor->setSpeed(motorSpeed);
}



}
