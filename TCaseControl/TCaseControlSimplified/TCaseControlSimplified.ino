/******************************************************************************

Name:         TCaseControlSimplified

Description:  Sketch running the transfer case controller on the Fummins.
              Reads the position of the dash switch (2WD, 4HI, 4LOW), and
              moves the motor of the electric shift transfer case (NV273)
              accordingly.  Uses the stock encoder on the transfer case for
              position feedback.

Platform:     MKR1010 WIFI

*******************************************************************************/
//#include <utility/wifi_drv.h> //For MKR1010WIFI RGB LED 

#define LOG 0  //All the verbose in the serial console
#define DEBUG_RAW_ENCODER 0 //Set to non-zero to debug raw encoder to console (LOG has to be also non-zero)

//Pinout (FOR ARDUINO MKR1010 WIFI)
#define ENCODER_1 5  //Encoder bit 1 / connector pin 5 (purple/yellow)
#define ENCODER_2 4  //Encoder bit 2 / connector pin 3 (brown/white)
#define ENCODER_3 3  //Encoder bit 3 / connector pin 6 (white)
#define ENCODER_4 2  //Encoder bit 4 / connector pin 1 (orange/white)

#define RELAY_POSITIVE 0  //pin that controls the relay that moves the motor to positive traction (orange motor wire)
#define RELAY_NEGATIVE 1  //pin that controls the relay that moves the motor to negative traction (yellow motor wire)

#define ANALOG_SWITCH_IN A1  //pin where we read the voltage coming back from the switch (voltage divider with other resistance)

//Target encoder values  (IF THERE IS A LEADING "0" YOU HAVE TO OMIT IT!!)
#define ENCODER_POS_2WD 111
#define ENCODER_POS_4HI 1010
#define ENCODER_POS_4LO 1101  //Check hall sensor output file for tweaking, other values possible here

//States
#define TRANSITION 0
#define _2WD 1
#define _4HI 2
#define _4LOW 3
#define IMPOSSIBLE 999

//Direction of movement
#define STOPPED 0
#define POSITIVE 1
#define NEGATIVE 2





/******************************************************************************

Function:     setup()

Arguments:    Nil

Description:  Called once initially to initialize all variables.

Return value: Nil

*******************************************************************************/
void setup()
{
  #if LOG == 1
  //Initialize serial:
  Serial.begin(115200);

  while (!Serial);  //Wait for serial port to be available

  lmsln("Debugging to serial enabled!");
  lmsln("Transfer case controller on MKR1010 WIFI");
  lmsln("Created for the Overland Fummins");
  lmsln("By Etienne Gignac-Bouchard");
  lmsln("December 2024");
#endif

  // Done setup, on to main loop
}


/******************************************************************************

  Function:     getRawEncoderValue()

  Description:  return the current value of the encoder as an int
                representation of the 4 bits
                e.g. 1010, 1101, 1111, etc.

  Returns:      int representation of the encoder value (leading zeros are
                omitted since it's an int)

*******************************************************************************/
int getRawEncoderValue()
{
  //Build return value, each digit is the value of one bit
  int rawEncoderValue = (digitalRead(ENCODER_1) * 1000) + (digitalRead(ENCODER_2) * 100) + (digitalRead(ENCODER_3) * 10) + digitalRead(ENCODER_4);

  return rawEncoderValue;
}


/******************************************************************************

  Function:     getTCaseStatus()

  Description:  translates the raw encoder value to actual status

  Possible return values:
      _2WD = 1
      _4HI = 2
      _4LOW = 3

*******************************************************************************/
int getTCaseStatus()
{
  switch(getRawEncoderValue()) //Get bit value from encoder
  {
    case ENCODER_POS_2WD:
      return _2WD;
      break;
    
    case ENCODER_POS_4HI:
      return _4HI;
      break;

    case ENCODER_POS_4LO:
      return _4LOW;
      break;

    default:
      return IMPOSSIBLE;
  }
}


/******************************************************************************

   Function: getRequestedState()

   Description: Reads the dash switch and returns the corresponding
   requested transfer case state

   Possible return values:
      _2WD = 1
      _4HI = 2
      _4LOW = 3

*******************************************************************************/
int getRequestedState()
{
  /* With external resistor R = 470
   *  and Vref = 5v (from supply regulator)
     Value of analogRead is:
       2WD: 582  \
                    476 cutoff
       4HI: 370  / 
                 \
                    243 cutoff
       4LO: 216  /

       Let's find the value in-between as limits(cutoffs).  Therefore: 
       2WD > 476
       476 > 4HI > 243
       10 < 4LO < 243
       IMPOSSIBLE < 0 (switched wiring probably unplugged)
  */

  //Read the position of the dash switch
  int switchPosition = analogRead(ANALOG_SWITCH_IN);

  //Figure out the position based on the values calculated as limits and return state
  // 0 = TRANSITION, 1 = 2WD, 2 = 4HI, 3 = 4LO, 999 = IMPOSSIBLE
  if (switchPosition > 511)
    return _2WD;

  else if (switchPosition <= 511 && switchPosition >= 319)
    return _4HI;

  else if (switchPosition < 319 && switchPosition > 10)
    return _4LOW;

  else
    return IMPOSSIBLE;
}


/******************************************************************************

   Function:    loop()

   Description: Main loop.  Every half second, check dash switch, check where
                transfer case is, compare the two.  Move to the right place.
                If we took too long, abort.

   Returns:     Nil
*******************************************************************************/
void loop()
{
  if(getRequestedState() != getTCaseStatus()) //If the transfer case doesn't match the dash switch
  {
    // Figure out if we need to go positive or negative...
    // Start timer
    // Start moving
    // If after timer end we are not there, abort
  }


  //Delay 500ms before looping again
  delay(500);
}
