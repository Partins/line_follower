/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
#include <DueTimer.h>
int task = 4;


volatile byte state = LOW;

int time1 = 0;
int time2 = 0;
int startTime = 0;
int endTime = 0;
// the setup function runs once when you press reset or power the board
/* Task 1*/
/* Install the Arduino IDE */  

/* Task 2 */
/* Blink with different frequencies */
/* Define a start frequency and choose a multiplier that increases
 *  the frequency after 20 blinks
 */
 
    int frequency = 1;
    int multiplier = 10; // For increasing the frequency of the blink

 /* Task 3 */
/* Create a program for counting pulse.  
 * Amount of counted pulses should be displayed in the Serial Monitor.
*/
    int rEnc = 2; // Right encoder Pin
    int lEnc = 3; // Left encoder pin
    int rPulseCntr = 0; // Right pulse counter
    int lPulseCntr = 0; // Left pulse counter
    int rCntr = 0;
    int lCntr = 0;
    bool rState = 0;
    bool lState = 0;

/* Task 4 */
/* Measure time between pulses */
    int numOfPulses = 10;
    int rTime = 0;
    int lTime = 0;
    int refTimeR = 0;
    int refTimeL = 0;
    double rPulsesPerSecond = 0;
    double lPulsesPerSecond = 0;
    double seconds = 1;
    double uTimerLength = 1000000*seconds;
    bool ledOn = false;


void intrR(){
  rCntr++;
  //rState = 1;
  
}

void intrL(){
  lCntr++;
  //lState = 1;
}

void timerInterrupt(){
 
    rPulsesPerSecond = rCntr/seconds;
    //rCntr = 0;

    lPulsesPerSecond = lCntr/seconds;
    

    rState = 1;
    

  //ledOn = !ledOn; For testing the interrupt
 
}

void setup() {
  Serial.begin(115200);
  
  switch (task) {
      case 1:
          Serial.println("Arduin IDE is installed");
          break;
      
      case 2: // Flashing LED
          pinMode(LED_BUILTIN, OUTPUT); 
          break; 

      case 3: // Counting pulses
          pinMode(rEnc, INPUT_PULLUP);
          pinMode(lEnc, INPUT_PULLUP);
          attachInterrupt(digitalPinToInterrupt(rEnc), intrR, RISING);
          attachInterrupt(digitalPinToInterrupt(lEnc), intrL, RISING);     
      break;

      case 4: // Pulse time
          pinMode(rEnc, INPUT_PULLUP);
          pinMode(lEnc, INPUT_PULLUP);
           pinMode(LED_BUILTIN, OUTPUT);
          attachInterrupt(digitalPinToInterrupt(rEnc), intrR, RISING);
          attachInterrupt(digitalPinToInterrupt(lEnc), intrL, RISING);  
          Timer3.attachInterrupt(timerInterrupt);
          Timer3.start(uTimerLength);   
      break;
      
  }
}

// the loop function runs over and over again forever
void loop() {
while(1){
  switch (task) {
    case 2: // Flashing LED
        for (int i = 0; i<20; i++){
            Serial.println("First loop");
            digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(1000/(frequency*multiplier));
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000/(frequency*multiplier));
        }
        multiplier = multiplier + 10;
    break;
        
    case 3: // Counting pulses
        if (rState == 1){
          Serial.print("Right pulses: ");
          Serial.println(rCntr);
          rState = 0;
        }
        if (lState == 1){
          Serial.print("Left pulses: ");
          Serial.println(lCntr);
          lState = 0;
        }       
    break;

    case 4:
    if(rState > 0){
        Serial.print("ms between pulses R: ");
        Serial.println(1000/rPulsesPerSecond,10);
        Serial.println(rCntr);
        Serial.print("Pulses per second L: ");
        Serial.println(1000/lPulsesPerSecond,10);
        Serial.println(lCntr);
        digitalWrite(LED_BUILTIN, ledOn);
        rState = 0;
        lCntr = 0;
        rCntr = 0;
        
    }
    break;

    
  }

}
}
//  if (pulseCounter == 1){
//    startTime = millis();
//    }
//    if (pulseCounter == 10){
//    endTime = millis(); 
//    }
//    
//  if (pulseCounter == 10){
//    Serial.print("Time between pulses");
//    Serial.println((endTime-startTime)/pulseCounter);
//    pulseCounter = 0;
//  }
//  Serial.print("pulseCounter: ");
//  Serial.println(pulseCounter);
//   if (pulseCounter == 10){
//    pulseCounter = 0;
//  }
