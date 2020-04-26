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
  float velocity;
  float timedPulses;
  bool forwardDirection;
}Wheel;

struct SensorBar {
  static const uint8_t kNumOfSensors{8};
  int calibrationValues[kNumOfSensors];
  bool calibrated = false;
  uint8_t sensorPins[8] = {A0, A1, A2, A7, A8, A9, A10, A11};
};

bool sensCalib(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor);
void turnLeft(int speed, Wheel *right, Wheel *left);
void turnRight(int speed, Wheel *right, Wheel *left);

#endif  // SRC_LAB4_H_
