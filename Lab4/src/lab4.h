/*
 *   Copyright (c) 2020 
 *   All rights reserved.
 */

#ifndef LAB4_H
#define LAB4_H
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

typedef struct {
  uint32_t counter;
  uint32_t prevCounter;  // Used to determine pulse increase
  uint8_t speed;
  const uint8_t MOTOR_NUMBER;
  Adafruit_DCMotor *dcMotor;
  double velocity;
  double velocityArr[5];
  int arrCounter;
  float timedPulses;
  int direction;  // Either FORWARD or BACKWARD
}Wheel;

typedef struct {
  float pGain;
  float iGain;
  float dGain;
  float tot_integral;
  float derivative;
  float pGainAngle;
  float iGainAngle;
  float dGainAngle;
  float tot_integralAngle;
  float derivativeAngle;
}Controller;

struct SensorBar {
  static const uint8_t kNumOfSensors{8};
  int calibrationValues[kNumOfSensors];
  int sensorValues[kNumOfSensors];
  bool lineDetected[kNumOfSensors];
  bool calibrated = false;
  float angle;
  float angleSetpoint;
  float error;
  int sensorSum;
  int cntr;
  // uint8_t sensorPins[8] = {A0, A1, A2, A7, A8, A9, A10, A11};  // Analog
  uint8_t sensorPins[8] = {6, 7, 8, 9, 10, 11, 12, 13};  // Digital
  uint8_t weights[8] = {10, 20, 30, 40, 50, 60, 70 ,80};
};


void sensCalib(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor);
void turnLeft(int speed, Wheel *rightWheel, Wheel *leftWheel);
void turnRight(int speed, Wheel *rightWheel, Wheel *leftWheel);
void readAllSensorsAnalog(SensorBar *sensor);
void readAllSensorsDigital(SensorBar *sensor);
void runMotors(Wheel *rightWheel, Wheel *leftWheel);
void calculateVelocity(Wheel *rightWheel, Wheel *leftWheel, float seconds);
void calculateTheta(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor);
void goStraight(Wheel *rightWheel, Wheel *leftWheel);
void piController(float setpoint, Controller *controller, Wheel *wheel);
void angleController(int setpoint, SensorBar *sensor, Controller *controller);

#endif  // SRC_LAB4_H_
