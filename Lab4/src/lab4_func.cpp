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

void readAllSensorsAnalog(SensorBar *sensor) { 
    for (int i = 0; i < sensor->kNumOfSensors; i++) {
        sensor->sensorValues[i] = analogRead(sensor->sensorPins[i]);
        if (sensor->sensorValues[i] > sensor->calibrationValues[i]*0.7) {
            sensor->lineDetected[i] = true;
        } else {
            sensor->lineDetected[i] = false;
        }
    }
}

void readAllSensorsDigital(SensorBar *sensor) {
    int tmp = 0;
    for (int i = 0; i < sensor->kNumOfSensors; i++) {
        tmp = digitalRead(sensor->sensorPins[i]);
        if (tmp == 1) {
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

void calculateTheta(Wheel *rightWheel, Wheel *leftWheel, SensorBar *sensor) {
    float thetaDot = 0;
    thetaDot = (rightWheel->velocity - leftWheel->velocity) / 0.153;
    sensor->angle = sensor->angle + thetaDot;
}

void piController(float setpoint, Controller *controller, Wheel *wheel) {
    float error = setpoint-wheel->velocity;
    controller->tot_integral += error;
    wheel->speed = (controller->pGain*error+controller->iGain*controller->tot_integral)*255;

}

void angleController(int setpoint, SensorBar *sensor, Controller *controller) {
    sensor->sensorSum = 0;
    int cntr = 0;
    float error = 0;
    for (int i = 0; i < sensor->kNumOfSensors; i++) {
        if (sensor->lineDetected[i] == true) {
        sensor->sensorSum = sensor->sensorSum+sensor->weights[i];
        cntr++;
    }
     error = setpoint-sensor->sensorSum/cntr;
     error = error*controller->pGain+error*controller->iGainAngle;
     sensor->angleSetpoint = error;
     sensor->cntr = cntr;
    }
}



