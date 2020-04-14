#include <QTRSensors.h>

QTRSensors qtr;
int SensorCount = 8;
int sensorValues[SensorCount];


void setup() {
  // put your setup code here, to run once:
  qtr.setTypeAnalog();
  qtr.setSensorPins((int[]){A3, A4, A5, A6, A7, A8, A9, A10, A11});
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}
