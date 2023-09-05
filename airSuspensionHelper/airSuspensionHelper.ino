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
- Sketch: airSuspensionHelper
- Author: Etienne Gignac-Bouchard
- Data started: 4 Sep 23
- Last modified: 4 Sep 23
- Description: Program used to implement other air suspension functions to supplement the K-sports main controller.

*/





/***********************************************

        GLOBAL VARIABLES

************************************************/


/***********************************************
Need:

4x analog input for bag pressure sensors (tap into existings), can be A0 to A15
4x analog input for air suspension level sensors (maybe share with main controller) can be A0 to A15
8 analog inputs means this will have to run on a Mega

8x digital out for 4 bags up/down solenoids through realy board, D0 to D19
SPI to level sensor, SDA/SCL on D20, D21

Modbus:
registers for
pressure in each bag
pitch angle
roll angle
maybe display all that on the Pi up front

        HARDWARE PIN MAPPING


              INPUTS
      (PIN)         (Description)
      A0            Analog input from pedal TPS (foot off = 0V).  Has to be able to to analog to digital.
                    


              OUTPUTS
      (PIN)         (Description)
      3            PWM Output to servo

************************************************/
#define pedal A0
#define sPin 3

/***********************************************

        SETUP

************************************************/
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(pedal, INPUT); //Configure pedal pin as input, add PULLUP or PULLDOWN as failsafe to pull towards no fuel (TODO check on truck)
  s.attach(sPin);    //Attach the servo to an output pin, apparently no need to manage the mode of the pin first
  s.write(sPosCur); //Send servo to initial position (zero)
}

/***********************************************

         LOOP

- Do we need to autolevel? (check modbus from RPI), yes, autolevel.  No, continue
- Manual buttons directly from relay board
- loop to read pressures and put values in registers

************************************************/
void loop()
{

  
}