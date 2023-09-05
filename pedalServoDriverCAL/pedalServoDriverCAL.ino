#include <Servo.h>

#define LOG_DEBUG 1 //Change this for 1 to enable serial monitor debugging
#if LOG_DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define NOT_MEASURED 9999

/*
- Sketch: pedalServoDriverCAL
- Author: Etienne Gignac-Bouchard
- Data started: 3 Sep 23
- Last modified: 3 Sep 23
- Description: Program used to determine which min and max values of the servo control the fuel pedal and which min and max 
               voltages are used in the pedal TPS.

*/





/***********************************************

        GLOBAL VARIABLES

************************************************/
Servo s;          //The servo object
long sPosMin = 9999;     //The position of the servo in degrees when fuel is completely off
long sPosMax = 9999;     //The position of the servo in degrees when fuel is max'ed
long sPosCur = 0;       // Holds current position of the servo, initialize to 0
long fuelPosMin = NOT_MEASURED;  //The position of the fuel pedal after analogRead (foot off the pedal)
long fuelPosMax = NOT_MEASURED;  //The position of the fuel pedal after analogRead (foot off the pedal)
char menuSelection; //Var used to store the input from user

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
  Serial.begin(115200);
  debugln("Setting up...");
  pinMode(pedal, INPUT); //Configure pedal pin as input, add PULLUP or PULLDOWN as failsafe to pull towards no fuel (TODO check on truck)
  s.attach(sPin);    //Attach the servo to an output pin, apparently no need to manage the mode of the pin first
  s.write(sPosCur); //Send servo to initial position (zero)
}

/***********************************************

         LOOP

************************************************/
void loop() {

  debugln("---------------------  Pedal Servo Driver Calibration ---------------------");
  debugln("");
  debugln("Current values:");
  
  debug("Fuel pedal voltage (foot off): ");
  if(fuelPosMin == NOT_MEASURED) debugln("NOT MEASURED");
  else { debug(fuelPosMin); debugln(" (10-bit ADC value)");}

  debug("Fuel pedal voltage (foot to the floor): ");
  if(fuelPosMax == NOT_MEASURED) debugln("NOT MEASURED");
  else { debug(fuelPosMax); debugln(" (10-bit ADC value)");}

  debug("Servo position (angle) - (fuel off - idle): ");
  if(sPosMin == NOT_MEASURED) debugln("NOT MEASURED");
  else { debug(sPosMin); debugln(" degrees"); }

  debug("Servo position (angle) - (fuel max'ed): ");
  if(sPosMax == NOT_MEASURED) debugln("NOT MEASURED");
  else { debug(sPosMax); debugln(" degrees"); }

  debug("Current servo angle: ");
  debug(sPosCur);
  debugln(" degrees");

  debugln("");
  debugln("");
  debugln("1. Read and set minimum pedal voltage (foot off)");
  debugln("2. Read and set maximum pedal voltage (foot to the floor)");
  debugln("3. Increase servo angle (more fuel)");
  debugln("4. Decrease servo angle (less fuel)");
  debugln("5. Set servo value as min value (idle)");
  debugln("6. Set servo value as max value (full fuel)");
  debugln("7. Output final values");
  debugln("---------------------------------------------------------------------------");
  debugln("");
  debugln("");
  debugln("");
  debugln("");


  //Wait for input
  while(!Serial.available()) {}

  // send data only when you receive data:
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    menuSelection = Serial.read();

    // make sure what we got is valid (1 through 6)
    switch (menuSelection)
    {
        case '1': //Read and set minimum fuel pedal voltage (foot off)
        fuelPosMin = analogRead(pedal); 
        debug("10-bit value set to: ");
        debug(fuelPosMin);
        debugln("/1024");
        break;

        case '2':
        fuelPosMax = analogRead(pedal); 
        debug("10-bit value set to: ");
        debug(fuelPosMax);
        debugln("/1024");
        break;

        case '3':
        sPosCur++;
        s.write(sPosCur);
        break;

        case '4':
        sPosCur--;
        s.write(sPosCur);
        break;

        case '5':
        sPosMin = sPosCur;
        debug("Minimum servo value set to: ");
        debug(sPosMin);
        debugln(" degrees.");
        break;

        case '6':
        sPosMax = sPosCur;
        debug("Maximum servo value set to: ");
        debug(sPosMax);
        debugln(" degrees.");
        break;

        case '7':
        debugln("FINAL VALUES");
        debug("sPosMin = ");
        debugln(sPosMin);

        while(true){} //Freeze program there
        break;
    }
  }
}