/*
 *   Copyright (c) 2020 
 *   All rights reserved.
 */

#include "lab4.h"


void turnRight(int speed, Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->speed = 0;
    leftWheel->speed = speed;
}

void turnLeft(int speed, Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->speed = speed;
    leftWheel->speed = 0;
    rightWheel->dcMotor->setSpeed(rightWheel->speed);
    leftWheel->dcMotor->setSpeed(leftWheel->speed);
    rightWheel->dcMotor->run(FORWARD);
    leftWheel->dcMotor->run(FORWARD);
}

void readAllSensors(SensorBar *sensor) {
    uint16_t tmp = 0;
    for (int i = 0; i < sensor->kNumOfSensors; i++) {
        tmp = analogRead(sensor->sensorPins[i]);
        if (tmp > sensor->calibrationValues[i]) {
            sensor->calibrationValues[i] = tmp;
        }
    }
}

bool sensCalib(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor) {
    turnLeft(50, rightWheel, leftWheel);
    
    readAllSensors(sensor);
    turnRight(50, rightWheel, leftWheel);
    readAllSensors(sensor);
return true;
}
