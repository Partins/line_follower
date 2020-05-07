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
        if (sensor->sensorValues[i] > sensor->calibrationValues[i]*0.8) {
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
/* 
void calculateVelocityDouble(Wheel *rightWheel, Wheel *leftWheel, float seconds) {
    
    rightWheel->velocity = 0.12566*rightWheel->timedPulses/(618*seconds);
    leftWheel->velocity = 0.12566*leftWheel->timedPulses/(618*seconds);
} */

void calculateVelocity(Wheel *wheel, float seconds) {

    wheel->velocity = 0.12566*wheel->timedPulses/(618*seconds);

}

void piController(float setpoint, Controller *controller, Wheel *wheel) {
    if (setpoint > 1){
        setpoint = 1;
    } else if (setpoint < 0) {
        setpoint= 0;
    }
    float error = setpoint - wheel->velocity;
    controller->error = error;
    controller->tot_integral += error;
    wheel->speed = (controller->pGain*error + controller->iGain*controller->tot_integral) * 255;
}


void findAngle(SensorBar *sensor) {
    int cntr = 0;
    float theta = 0;
    //float error = 0;
    for (int i = 0; i<sensor->kNumOfSensors; i++) { 
        if (sensor->lineDetected[i] == true) {
            theta = theta + sensor->sensorAngles[i];
            cntr++;
        }
    }
    if (cntr == 0) {  // To not divide by 0
        cntr = 1;
    }
    sensor->thetaDot = (theta/cntr - sensor->theta);
    sensor->theta = theta / cntr;
    
}

void findGoalPosition(SensorBar *sensor) {
    float prevYerrorDot = sensor->errorDot;
    float prevY = sensor->goalY;
    float pGain = sensor->positionP;
    float iGain = sensor->positionI;
    float dGain = sensor->positionD;
    int cntr = 0;
    sensor->goalY = 0;
    
    for (int i = 0; i<sensor->kNumOfSensors; i++) { 
        if (sensor->lineDetected[i] == true) {
            sensor->goalY = sensor->sensorDistances[i]/(119);
            cntr=1;
        }
    }
    if(cntr == 0) {
        //sensor->goalY += prevY;
        sensor->error = prevY;
        sensor->errorInt += prevY;
        sensor->errorDot = sensor->errorDot - prevY;
        cntr++;
    } else {
    sensor->goalY = sensor->goalY/cntr;
    sensor->errorInt += sensor->goalY;
    sensor->errorDot = sensor->goalY - prevYerrorDot;
    }
    sensor->velocitySetpoint = pGain * sensor->goalY   + iGain * sensor->errorInt + dGain * sensor->errorDot;
}