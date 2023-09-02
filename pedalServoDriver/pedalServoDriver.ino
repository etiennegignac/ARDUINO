#include <Servo.h>

#define VALIDATE_WITHOUT_HARDWARE 1 //change this to 1 to use software random number generator instead of actual TPS

#define LOG_DEBUG 0 //Change this for 1 to enable serial monitor debugging
#if LOG_DEBUG == 1 || VALIDATE_WITHOUT_HARDWARE == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

/*
- Sketch: pedalServoDriver
- Author: Etienne Gignac-Bouchard
- Data started: 2 Sep 23
- Last modified: 2 Sep 23
- Description: Reads the TPS on the stock fuel pedal and drives the servo motor accordingly

*/





/***********************************************

        GLOBAL VARIABLES

************************************************/
Servo s;          //The servo object
int sPos = 0;     //The position in degrees from 0 to 359, initialize to 0, should be fuel completely off
int fuelPos = 0;  //The position of the fuel pedal after analogRead
int sMax;         //Max servo position corresponding to full fuel
int sMin;         //Min servo position corresponding to no fuel





/***********************************************

        HARDWARE PIN MAPPING


              INPUTS
      (PIN)         (Description)
      A0            Analog input from pedal TPS (foot off = 0V).  Has to be able to to analog to digital.
                    On Arduino UNO, can be A0 to A5


              OUTPUTS
      (PIN)         (Description)
      3            PWM Output to servo

            SERVO PINOUT (TYPICAL)
      RED
      YELLOW
      BROWN         GROUND

************************************************/
#define pedal A0
#define sPin 3





/***********************************************

        SETUP

************************************************/
void setup() {
  // put your setup code here, to run once:
  debug("Setting up...");
  pinMode(pedal, INPUT); //Configure pedal pin as input, add PULLUP or PULLDOWN as failsafe to pull towards no fuel (TODO check on truck)
  s.attach(sPin);    //Attach the servo to an output pin, apparently no need to manage the mode of the pin first
}





/***********************************************

         LOOP

************************************************/
void loop() {
  // put your main code here, to run repeatedly:

  // Read the analog position of the pedal
  fuelPos = analogRead(pedal);
  // Calculate corresponding servo position
  // fuelPos / 1024 = sPos / (sMax - sMin), solve for sPos gives:
  sPos = fuelPos * (sMax - sMin) / 1024; 

  // Write calculated position to servo
  
  
  //s.write(sPos);

  //Maybe a quick delay
}
