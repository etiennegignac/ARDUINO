/*
- Sketch: pedalServoDriver
- Author: Etienne Gignac-Bouchard
- Data started: 13 Jan 24
- Last modified: 13 Jan 24
- Description: Reads the TPS on the stock fuel pedal and drives the servo motor accordingly

Controller: Arduino UNO (could be adapter to anything that has a 5V logic level)
*/

#include <Servo.h>


#define VALIDATE_WITHOUT_HARDWARE 1 //change this to 1 to use software random number generator instead of actual TPS

#define LOG 1 //Change this for 1 to enable serial monitor debugging
#if LOG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

//#if VALIDATE_WITHOUT_HARDWARE == 1
//#define analogRead(CALIBRATE_PIN) generateRandomTPSValue()
//#endif

/***********************************************

        GLOBAL VARIABLES

************************************************/
Servo s;                  //The servo object
long servoPos = 0;        //The position in degrees from 0 to 359, initialize to 0, should be fuel completely off
long fuelPos = 0;         //The position of the fuel pedal after analogRead
long minFuel = 0;         //Min value of the fuel pedal
long maxFuel = 0;         //Max value of the fuel pedal
int servoMin = 0;         //Min servo position (angle) corresponding to no fuel
int servoMax = 90;        //Max servo position (angle) corresponding to full fuel


/***********************************************

        HARDWARE PIN MAPPING


              INPUTS
      (PIN)         (Description)
      A0            Analog input from pedal TPS (foot off = 0V).  Has to be able to to analog to digital.
      2             Input from calibrate button.  Button brings this input to ground when pushed.  This has to have a pullup resistor.
      

              OUTPUTS
      (PIN)         (Description)
      3            PWM Output to servo

            SERVO PINOUT (TYPICAL)
      RED           Positive (check servo supply voltage)
      YELLOW        Signal
      BROWN         GROUND



************************************************/
#define PEDAL_PIN A0    //Voltage input from pedal signal
#define SERVO_PIN 3     //PWM output to servo
#define CALIBRATE_PIN 2 //Also used as "up" button
#define DOWN_PIN 4      //"Down" button



/***********************************************

        SETUP

************************************************/
void setup() {
  
  //Setup pin modes
  pinMode(PEDAL_PIN, INPUT);     //voltage from pedal
  pinMode(CALIBRATE_PIN, INPUT_PULLUP);   //Input from calibrate/up button.  Pulled up to Vcc, grounded when pushed.
  pinMode(DOWN_PIN, INPUT_PULLUP);        //Input for second button ("Down").  Pulled to Vcc,grounded when pushed
  pinMode(SERVO_PIN, OUTPUT);             //Servo output
  pinMode(LED_BUILTIN, OUTPUT);           //LED for feedback

#if LOG == 1 //We want logged output
  Serial.begin(115200);
  debugln("Setting up...");
#endif

  //Flash LED once to signal you have to have the button pushed if yo uwant to go into calibrate
  digitalWrite(LED_BUILTIN, HIGH);         //HIGH is on
  delay(5000); //5 sec delay

  //Check status of input pin after 5 secs
  if(!digitalRead(CALIBRATE_PIN)) //If our calibrate pin is low (button is pushed)
  {
      debugln("Calibration button pushed.");
      //Flash LED twice to signal calibration mode
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);

      //Enter calibration mode
      doCalibration();
  }

  else //Calibrate button was not pushed.
  {
    debugln("Calibration button not pushed.");
    //Turn off LED and carry on
    digitalWrite(LED_BUILTIN, LOW);
  }

  //Attach servo output to servo pin
  s.attach(SERVO_PIN);

  //Put servo at zero position
  s.write(0); //TODO this should be replaced with the value from the pedal at 0 fuel.

}

void loop() {
  // put your main code here, to run repeatedly:

}


/***********************************************

        doCalibration()

- Desciption: used to get pedal voltages and calibrate servo min and max positions to use in this software
- Arguments: none
************************************************/
void doCalibration()
{
  //Wait for calibration button to be released so that we don't get confused
  while(!digitalRead(CALIBRATE_PIN));

#if LOG == 1
  debugln("Entering calibration mode.........");
#endif

  while(true) //Step : pedal minimum voltage
  {
    //Flash led slowly once.  If calibrate button is pressed at the end of the loop, move to second step.  If not, try again.
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2000);

    if(!digitalRead(CALIBRATE_PIN)) //Calibrate button is pressed, store value and move on
    {
      //minFuel = analogRead(PEDAL_PIN);
      minFuel = generateRandomTPSValue();
      break;
    }
  }

  //Wait for calibration button to be released so that we don't get confused
  while(!digitalRead(CALIBRATE_PIN));

#if LOG == 1
  debugln("Exiting calibration mode with values: ");
  debug("Fuel pedal minimum: ");
  debugln(minFuel);
  debugln("--------------------------------------");
#endif

}

#if VALIDATE_WITHOUT_HARDWARE == 1 //We only need this function if we don't have hardware connected to Arduino
/***********************************************

        generateRandomTPSValue()

- Desciption: used to generate a random value when we do not have a TPS pedal connected.  Uses this random number instead of reading the pedal value
- Arguments: none
- Returns: int value max 0 to 1023
************************************************/
long generateRandomTPSValue() {
        randomSeed(analogRead(0));
        return random(0, 1023); // returns a random number between 0 and 1023
}
#endif
