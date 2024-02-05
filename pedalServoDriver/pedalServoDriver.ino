/*
- Sketch: pedalServoDriver
- Author: Etienne Gignac-Bouchard
- Data started: 13 Jan 24
- Last modified: 13 Jan 24
- Description: Reads the TPS on the stock fuel pedal and drives the servo motor accordingly

*/



#include <Servo.h>
#include <ArduinoOTA.h> //OTA

#define VALIDATE_WITHOUT_HARDWARE 0 //change this to 1 to use software random number generator instead of actual TPS

#define LOG_DEBUG 1 //Change this for 1 to enable serial monitor debugging
#if LOG_DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

/***********************************************

        GLOBAL VARIABLES

************************************************/
Servo s;          //The servo object
long sPos = 0;     //The position in degrees from 0 to 359, initialize to 0, should be fuel completely off
long fuelPos = 0;  //The position of the fuel pedal after analogRead
int sMax = 90;         //Max servo position (angle) corresponding to full fuel
int sMin = 0;         //Min servo position (angle) corresponding to no fuel





/***********************************************

        HARDWARE PIN MAPPING


              INPUTS
      (PIN)         (Description)
      A0            Analog input from pedal TPS (foot off = 0V).  Has to be able to to analog to digital.

      

              OUTPUTS
      (PIN)         (Description)
      3            PWM Output to servo

            SERVO PINOUT (TYPICAL)
      RED           
      YELLOW
      BROWN         GROUND



************************************************/
#define PEDAL_PIN A0
#define SERVO_PIN 3



#if VALIDATE_WITHOUT_HARDWARE == 1 //We only need this function if we don't have hardware connected to Arduino
/***********************************************

        generateRandomTPSValue()

- Desciption: used to generate a random value when we do not have a TPS pedal connected.  Uses this random number instead of reading the pedal value
- Arguments: none
- Returns: int value max 0 to 1023
************************************************/
long generateRandomTPSValue() {
        return random(0, 1023); // returns a random number between 0 and 1023
}
#endif



/***********************************************

        SETUP

************************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  debugln("Setting up...");
  pinMode(pedal, INPUT); //Configure pedal pin as input, add PULLUP or PULLDOWN as failsafe to pull towards no fuel (TODO check on truck)
  s.attach(sPin);    //Attach the servo to an output pin, apparently no need to manage the mode of the pin first
  s.write(sPos); //Send servo to initial position (zero)
}





/***********************************************

         LOOP

************************************************/
void loop() {
  // put your main code here, to run repeatedly:

  // Read the analog position of the pedal (or generate random value)
#if VALIDATE_WITHOUT_HARDWARE == 1 //We don't have the fuel pedal connected, used a random number instead
  fuelPos = generateRandomTPSValue();
  debug("Generated random fuelPos ");
  debugln(fuelPos);
  int percentA = fuelPos * 100 / 1024; // Calculate percentage to compare

#else //We have the fuel pedal connected, use actual value as input
  fuelPos = analogRead(pedal);
  debug("Read actual fuel position value of ");
  debugln(fuelPos);
#endif

  // Calculate corresponding servo position
  // fuelPos / 1024 = sPos / (sMax - sMin), solve for sPos gives:
  sPos = fuelPos * (sMax - sMin) / 1024;
  int percentB = sPos * 100 / (sMax - sMin); 

  debug("Calculated servo position is ");
  debugln(sPos);

#if VALIDATE_WITHOUT_HARDWARE == 1 // Use calculated percentage to compare and validate
  debug("Validating calculations and percentage error <1% : ");
  debug(percentA);
  debug(":");
  debug(percentB);
  debug(" ------------------- ");
  if(abs(percentA - percentB) > 1) debugln("FAIL!");
  else debugln("PASS!");

#else //We don't have hardware assumes we don't have servo output either
  // Write calculated position to servo
  s.write(sPos);
#endif

  //Maybe a quick delay
  delay(100);
}
