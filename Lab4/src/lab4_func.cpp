#include "lab4.h"

void runMotors(Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->dcMotor->setSpeed(rightWheel->speed);
    leftWheel->dcMotor->setSpeed(leftWheel->speed);
    rightWheel->dcMotor->run(FORWARD);
    leftWheel->dcMotor->run(FORWARD);
}

void turnRight(int speed, Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->speed = 0;
    leftWheel->speed = speed;
    runMotors(rightWheel, leftWheel);
}

void turnLeft(int speed, Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->speed = speed;
    leftWheel->speed = 0;
    runMotors(rightWheel, leftWheel);
}

void readAllSensors(SensorBar *sensor) {
    uint16_t tmp = 0;
    uint16_t calibrationCounter = 0;
    uint16_t tmpStart[sensor->kNumOfSensors];

    for (int i = 0; i < sensor->kNumOfSensors; i++) {
        tmpStart[i] = analogRead(sensor->sensorPins[i]);
    }
    while (calibrationCounter < 2) {
        for (int i = 0; i < sensor->kNumOfSensors; i++) {
            tmp = analogRead(sensor->sensorPins[i]);
            if (tmp > sensor->calibrationValues[i] && tmp > tmpStart[i]*5) {
                sensor->calibrationValues[i] = tmp;
                if (i == 1 || i == 8)
                    calibrationCounter++;
            }
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
