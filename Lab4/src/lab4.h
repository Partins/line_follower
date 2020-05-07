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
  float setpointVelocity;
  double velocityArr[5];
  int arrCounter;
  float timedPulses;
  int direction;  // Either FORWARD or BACKWARD
}Wheel;

typedef struct {
  float xGoal;
  float yGoal;
  float velocity;
  float heading;
} Setpoint;

typedef struct {
  float pGain;
  float iGain;
  float dGain;
  float tot_integral;
  float totDerivative;
  float derivative;
  float pGainAngle;
  float error;
}Controller;

struct SensorBar {
  static const uint8_t kNumOfSensors{16};
  int calibrationValues[kNumOfSensors];
  int sensorValues[kNumOfSensors];
  bool lineDetected[kNumOfSensors];
  bool calibrated = false;
  float angle;
  float angleSetpoint;
  float error;
  float errorDot;
  int sensorSum;
  int cntr;
  float d;
  float goalAngle;
  float goalX;
  float goalY;
  float x;
  float y;
  float xDot;
  float yDot;
  float thetaDot;
    float positionP;
  float positionI;
  float positionD;
  float velocity;
  float errorInt;
  float theta;
  float velocitySetpoint;
   //uint8_t sensorPins[8] = {A0, A1, A2, A7, A8, A9, A10, A11};  // Analog
  uint8_t sensorPins[16] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};  // Digital
  //int8_t weights[8] = {20, 15, 10, 5, -5, -20, -15, -20};
  float sensorAngles[16] = {26.7, 23.6, 20.3, 16.8, 13.2, 9.5, 5.8, 1.9, -1.9, -5.8, -9.5, -13.2, -16.8, -20.3, -23.6, -26.7};
  float sensorDistances[16] = {60, 52, 44, 36, 28, 20, 12, 4, -4, -12, -20, -28, -36, -44, -52, -60}; // Digital
  //float sensorDistances[8] = {28, 20, 12, 4, -4, -12, -20, -28};  // Analog 12-5
  //float sensorDistances[8] = {60, 36, 20, 4, -4, -20, -36, -60};  // Analog something

  
};


void sensCalib(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor);
void turnLeft(int speed, Wheel *rightWheel, Wheel *leftWheel);
void turnRight(int speed, Wheel *rightWheel, Wheel *leftWheel);
void readAllSensorsAnalog(SensorBar *sensor);
void readAllSensorsDigital(SensorBar *sensor);
void runMotors(Wheel *rightWheel, Wheel *leftWheel);
void calculateVelocity(Wheel *wheel, float seconds);
void calculateTheta(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor, float seconds);
void goStraight(Wheel *rightWheel, Wheel *leftWheel);
void piController(float setpoint, Controller *controller, Wheel *wheel);
void angleController(int setpoint, SensorBar *sensor, Controller *controller);
void findThetaOffset(SensorBar *sensor);
void angleSpeedController(float setpoint, Wheel *rightWheel, Wheel *leftWheel, Controller *controller, SensorBar *sensor, float seconds);
void findAngle(SensorBar *sensor);
void findGoalPosition(SensorBar *sensor);
#endif  // SRC_LAB4_H_
