#include "lab4.h"

void runMotors(Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->dcMotor->setSpeed(rightWheel->speed);
    leftWheel->dcMotor->setSpeed(leftWheel->speed);
    rightWheel->dcMotor->run(rightWheel->direction);
    leftWheel->dcMotor->run(leftWheel->direction);
}

void stopMotors(Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->dcMotor->setSpeed(0);
    leftWheel->dcMotor->setSpeed(0);
}

void releaseMotors(Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->dcMotor->run(RELEASE);
    leftWheel->dcMotor->run(RELEASE);
}

void turnRight(int speed, Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->speed = speed;
    leftWheel->speed = speed;
    rightWheel->direction = BACKWARD;
    leftWheel->direction = FORWARD;
    runMotors(rightWheel, leftWheel);
}

void turnLeft(int speed, Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->speed = speed;
    leftWheel->speed = speed;
    rightWheel->direction = FORWARD;
    leftWheel->direction = BACKWARD;
    runMotors(rightWheel, leftWheel);
}

void goStraight(Wheel *rightWheel, Wheel *leftWheel) {
    rightWheel->direction = FORWARD;
    leftWheel->direction = FORWARD;
}

void readAllSensors(SensorBar *sensor) {
    for (int i = 0; i < sensor->kNumOfSensors; i++) {
        sensor->sensorValues[i] = analogRead(sensor->sensorPins[i]);
        if (sensor->sensorValues[i] > sensor->calibrationValues[i]*0.7) {
            sensor->lineDetected[i] = true;
        } else {
            sensor->lineDetected[i] = false;
        }
    }
}

void sensCalib(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor) {
    uint16_t tmp = 0;
    for (int j = 0; j < 90; j++) {
        for (int i = 0; i< sensor->kNumOfSensors; i++) {
            tmp = analogRead(sensor->sensorPins[i]);
            delay(1);
            if (tmp > sensor->calibrationValues[i]) {
                sensor->calibrationValues[i] = tmp;
            }
        }
    }
}

void calculateVelocity(Wheel *rightWheel, Wheel *leftWheel, float seconds) {
    rightWheel->velocity = 0.12566*rightWheel->timedPulses/(618*seconds);
    leftWheel->velocity = 0.12566*leftWheel->timedPulses/(618*seconds);
}

void pController(float setpoint, float pGain, Wheel *rightWheel, Wheel *leftWheel) {
    float rightError = 0;
    rightError = setpoint-rightWheel->velocity;
    rightWheel->speed = pGain*rightError*255;
}
void piController(float setpoint, float pGain, float iGain, Controller *controller, Wheel *rightWheel, Wheel *leftWheel) {
    float error = setpoint-rightWheel->velocity;
    controller->tot_integral += error;
    rightWheel->speed = (pGain*error+iGain*controller->tot_integral)*255;

}
