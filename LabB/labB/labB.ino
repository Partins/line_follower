#include <QTRSensors.h>


QTRSensors qtr;
const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int maxSensorValues[SensorCount];
static const uint8_t sensorPins[] = {A0, A1, A2, A7, A8, A9, A10, A11};
bool calibrationComplete = false;
int calibrationCounter = 0;
int tmp = 0;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A7, A8, A9, A10, A11}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
Serial.println("Calibration starting in 1s");
delay(1000);
while(calibrationCounter < 100000){
  for(int i = 0; i<8; i++){
    tmp = analogRead(sensorPins[i]);
    if(tmp > maxSensorValues[i]){
      maxSensorValues[i] = tmp;
    }
  }
 calibrationCounter++;
  
  }
  Serial.println("Maximum values");
 for(int i=0; i<8; i++){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" :");
    Serial.println(maxSensorValues[i]);   
 };
 delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:

  for(int i = 0; i<8; i++){
    sensorValues[i] = analogRead(sensorPins[i]);
    delay(1);
  }
 for(int i=0; i<8; i++){
   if(sensorValues[i] >= maxSensorValues[i] * 0.6){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" :");
    Serial.print(sensorValues[i]); 
    Serial.println("Line detected");
    }
 };
 
}
