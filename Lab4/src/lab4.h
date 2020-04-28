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
  float timedPulses;

  int direction;  // Either FORWARD or BACKWARD
}Wheel;

typedef struct {
  float pGain;
  float iGain;
  float dGain;
  float tot_integral;
  float derivative;
}Controller;

struct SensorBar {
  static const uint8_t kNumOfSensors{8};
  int calibrationValues[kNumOfSensors];
  int sensorValues[kNumOfSensors];
  bool lineDetected[kNumOfSensors];
  bool calibrated = false;
  uint8_t sensorPins[8] = {A0, A1, A2, A7, A8, A9, A10, A11};
};

void sensCalib(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor);
void turnLeft(int speed, Wheel *rightWheel, Wheel *leftWheel);
void turnRight(int speed, Wheel *rightWheel, Wheel *leftWheel);
void readAllSensors(SensorBar *sensor);
void runMotors(Wheel *rightWheel, Wheel *leftWheel);
void calculateVelocity(Wheel *rightWheel, Wheel *leftWheel, float seconds);
void goStraight(Wheel *rightWheel, Wheel *leftWheel);
void pController(float setpoint, float pGain, Wheel *rightWheel, Wheel *leftWheel);
void piController(float setpoint, float pGain, float iGain, Controller *controller, Wheel *rightWheel, Wheel *leftWheel);

#endif  // SRC_LAB4_H_
