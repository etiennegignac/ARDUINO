/*
   Author: Etienne Gignac-Bouchard, etiennegignac@hotmail.com
   Date: 19 Oct 22


   Implementation of Electric Shift On the Fly (ESOF) of a NV273F
   transfer case out of a 2000 Ford F350 in a 2012 Ford F550

   Hardware:  Arduino MKR 1010 wifi (obsolete) ---> Ported to WEMOS D1 mini
              Chinese Relay board (at least 2 relays on it)
              2 x automotive relays

 *********************************************************************
   Transfer case connectors pinout
 *********************************************************************

  Encoder:
    pin 1: orange/white
    pin 2: orange (not connected/used)
    pin 3: brown/white
    pin 4: purple/yellow
    pin 5: purple/yellow 
    pin 6: white 

  Motor connector
    pin 1: Yellow: positive when going for less traction
    pin 2: Orange: positive when going for more traction

 *********************************************************************
   Inputs/outputs (Check #define section below for pinout)
 *********************************************************************

   Arduino INPUTS:

              DIGITAL
              Encoder bit 1 - to connector pin 5 (purple/yellow)
              Encorer bit 2 - to connector pin 3 (brown/white)
              Encoder bit 3 - to connector pin 6 (white)
              Encoder bit 4 - to connector pin 1 (orange/white)

              connector pin 2 = +3.3v from board
              connector pin 4 = board ground 


              ANALOG
              Analog IN (from dash switch)

              DIAGRAM
                          Externak R = 470 ohms                      Dash switch
              3.3v    ----/\/\/\/\/\/\/\/\/\/\/-------------(pin 6)/\/\/\/\/\/\/\/(pin  7)--------- board
              from                                  |                                               ground
              board                                 |
                                                    |
                                                    input to analog IN (A0)
              INTERNAL REISTOR VALUES:
                According to service manual: 3 resistor in parallel between pins 6 and 7: 619 Ohms = 2WD, 270 Ohms = 4HI, 130 Ohms = 4LO

               
              SWITCH PINOUT (looking at back of switch, clip lock facing up)

                  |----|
            -----------------
            | 1  2  3  4  5  |   RESISTORS BETWEEN PINS 6 and 7
            | 6  7  8  9  10 |
            ------------------
              

   Arduino OUTPUTS:

              Output to increase traction (+) i.e. go from 2WD to 4HI to 4LOW (to control t-case orange motor wire)
              Output to decrease traction (-) i.e. go from 4LOW to 4HI to 2WD (to control t-case yellow motor wire)
*/


#define LOG 0  //All the verbose in the serial console
#define DEBUG_PIN_STATUS_ON_INTERRUPT 0
#define DEBUG_REQUESTED_STATE_PINS 0
#define DEBUG_CURRENT_STATE_PINS 0

// Little debug trick to strip the code of debug statement at compile time
#if LOG == 1
#define lms(x) Serial.print(x)
#define lmsln(x) Serial.println(x)

#else
#define lms(x)
#define lmsln(x)
#endif

//Pinout (FOR ARDUINO UNO)
#define ENCODER_1 2  //Encoder bit 1 / connector pin 5 (purple/yellow)
#define ENCODER_2 3  //Encoder pin 2 / connector pin 3 (brown/white)
#define ENCODER_3 4  //Encoder pin 3 / connector pin 6 (white)
#define ENCODER_4 5  //Encoder pin 4 / connector pin 1 (orange/white)

#define RELAY_POSITIVE 6  //pin that controls the relay that moves the motor to positive traction (orange motor wire)
#define RELAY_NEGATIVE 7  //pin that controls the relay that moves the motor to negative traction (yellow motor wire)


#define ANALOG_SWITCH_IN A0  //pin where we read the voltage coming back from the switch (voltage divider with other resistance)

//States
#define TRANSITION 0
#define _2WD 1
#define _4HI 2
#define _4LOW 3
#define IMPOSSIBLE 999

//Direction of movement
// POSITIVE = More traction (From 2WD to 4HI to 4LOW)
// NEGATIVE = Less traction (From 4LOW to 4HI to 2WD)
#define STOPPED 0
#define POSITIVE 1
#define NEGATIVE 2

//Global variables
int currentState = IMPOSSIBLE;  //Based on transfer case motor position
int requestedState = _2WD;      //Based on dash switch position, initialize as 2WD for calibration

int rawEncoderValue = 0;  //Used to store bit vlues as int representation (1011, 1111, etc)
int pastEncoderValue = 0;


int directionOfMovement = STOPPED;  //Motor direction of movement
bool isMotorMoving = false;         //true when motor is moving

int switchPosition = 0;  //Variable where we store the value of the analog to digital converter on ADC pin from switch




/******************************************************************

   Arduino setup

 *****************************************************************/
void setup() {

#if LOG == 1
  Serial.begin(115200);
  lmsln("Booting...");
#endif


  //All encoder pins as input pulled up
  pinMode(ENCODER_1, INPUT_PULLUP);
  pinMode(ENCODER_2, INPUT_PULLUP);
  pinMode(ENCODER_3, INPUT_PULLUP);
  pinMode(ENCODER_4, INPUT_PULLUP);

  //Outputs to relays
  pinMode(RELAY_POSITIVE, OUTPUT);
  pinMode(RELAY_NEGATIVE, OUTPUT);

  //Make sure motor is not running (for the relay board, HIGH is OFF)
  digitalWrite(RELAY_POSITIVE, HIGH);
  digitalWrite(RELAY_NEGATIVE, HIGH);

  //Start calibrating to make sure we get right values and positions
  calibrate();

#if LOG == 1
  lmsln("-----------------  Setup complete  -----------------");
#endif
}

/******************************************************************

   Function: getRawEncoderValue()

   Description: return the current value of the encoder as an int
                representation of the 4 bits
                e.g. 1010, 1101, 1111, etc.

  Updates global variable rawEncoderValue

 *****************************************************************/
int getRawEncoderValue() {
  //Build return value, each digit is the value of one bit
  rawEncoderValue = (digitalRead(ENCODER_1) * 1000) + (digitalRead(ENCODER_2) * 100) + (digitalRead(ENCODER_3) * 10) + digitalRead(ENCODER_4);

  return rawEncoderValue;
}


/******************************************************************

   Function: getRequestedState()

   Description: Reads the dash switch and returns the corresponding
   requested transfer case state

   Possible return values:
      _2WD = 1
      _4HI = 2
      _4LOW = 3

 *****************************************************************/
int getRequestedState() {
  /* With external resistor R = 470
   *  and Vref = 5v (from supply regulator)
     Value of analogRead is:
       2WD: 582  \
                    476 cutoff
       4HI: 370  / 
                 \
                    243 cutoff
       4LO: 216  /

       Let's find the value in-between as limits(cutoff).  Therefore: 
       2WD > 476
       476 > 4HI > 243
       10 < 4LO < 243
       IMPOSSIBLE < 0 (switched wiring probably unplugged)
  */

  //Read the position of the dash switch
  switchPosition = analogRead(ANALOG_SWITCH_IN);

  //Figure out the position based on the values calculated as limits
  if (switchPosition > 511)
    requestedState = _2WD;

  else if (switchPosition <= 511 && switchPosition >= 319)
    requestedState = _4HI;

  else if (switchPosition < 319 && switchPosition > 10)
    requestedState = _4LOW;

  else
    requestedState = IMPOSSIBLE;

#if DEBUG_REQUESTED_STATE_PINS
  lms("Requested state read from switch: ");
  lms(switchPosition);
  lms(" --> ");
  lmsln(requestedState);
#endif

  return requestedState;  // 0 = TRANSITION, 1 = 2WD, 2 = 4HI, 3 = 4LO, 999 = IMPOSSIBLE
}

/******************************************************************

   Function: calibrate():  

   Description: called initially to position the motor to the 2WD
                position regardless of dash switch.  Only way to know
                proper position without persistent storage.

 *****************************************************************/
void calibrate() {
  lmsln("Calibrating...");
  lmsln("Going to 2WD position.");

  //Start our recursive function
  find2WD();

  lmsln("Calibration finished.  Current position is 2WD.");
}

//recursive
void find2WD() {
  //lmsln("Looking for 1111 going negative...");

  //Turn motor on going negative
  digitalWrite(RELAY_NEGATIVE, LOW);

  //Wait a small amount of time
  delay(300);

  //See what we have
  if (getRawEncoderValue() == 1111)  //Looks like we were at the leftmost position already
  {
    //lmsln("Found leftmost 1111 encoder position.");
    digitalWrite(RELAY_NEGATIVE, HIGH);
  }

  else  //We are somewhere random
  {
    while (getRawEncoderValue() != 1111)  //Keep going until we find a 1111 position
    {
      //lms("Encoder: ");
      //lmsln(getRawEncoderValue());
      delay(10);
    }

    //lmsln("Found T-case position.");
    digitalWrite(RELAY_NEGATIVE, HIGH);  //Stop motor
    delay(100);
    find2WD();  //Call this function recursively
  }

  //Update current state
  currentState = _2WD;
}

void find4LO() {
  //lmsln("Looking for 1111 going positive...");

  //Turn motor on going positive
  digitalWrite(RELAY_POSITIVE, LOW);

  //Wait a small amount of time
  delay(300);

  //See what we have
  if (getRawEncoderValue() == 1111)  //Looks like we were at the leftmost position already
  {
    //lmsln("Found rightmost 1111 encoder position.");
    digitalWrite(RELAY_POSITIVE, HIGH);
  }

  else  //We are somewhere random
  {
    while (getRawEncoderValue() != 1111)  //Keep going until we find a 1111 position
    {
      //delay(10);
    }

    //lmsln("Found T-case position.");
    digitalWrite(RELAY_POSITIVE, HIGH);  //Stop motor
    delay(100);
  }

  //Update current state
  currentState = _4LOW;
}

void find4HIFrom4LO() {
  //Finf the second? 1010 from where we are

  digitalWrite(RELAY_POSITIVE, LOW);  //Make sure we start from the very end
  delay(100);                         //Don't need to long, we should already be there

  //Then we go negative
  digitalWrite(RELAY_POSITIVE, HIGH);  //Stop going positive
  digitalWrite(RELAY_NEGATIVE, LOW);   //Start going negative

  delay(250);  //Small delay to skip 4LO

  int numberOf1010s = 0;
  bool gotSomethingElseOtherThan1010 = true;

  while (numberOf1010s != 2) {
    if (getRawEncoderValue() == 1010 && gotSomethingElseOtherThan1010) {
      numberOf1010s++;  //Increase the number of times we encountered 1010 at the encoder
      //lms("Found 1010 #");
      //lmsln(numberOf1010s);
      gotSomethingElseOtherThan1010 = false;
    } else if (getRawEncoderValue() != 1010) {
      gotSomethingElseOtherThan1010 = true;
    }
    delay(10);
  }

  //Update current state
  currentState = _4HI;

  digitalWrite(RELAY_NEGATIVE, HIGH);  //Stop motor
}


void find4HIFrom2WD() {
  //Find the third? 1010 from where we are

  digitalWrite(RELAY_NEGATIVE, LOW);  //Make sure we start from the very end
  delay(100);                         //Don't need to long, we should already be there

  //Then we go negative
  digitalWrite(RELAY_NEGATIVE, HIGH);  //Stop going negative
  digitalWrite(RELAY_POSITIVE, LOW);   //Start going positive

  delay(250);  //Small delay to skip 2WD

  int numberOf1011s = 0;
  bool gotSomethingElseOtherThan1011 = true;

  while (numberOf1011s != 4) {
    if (getRawEncoderValue() == 1011 && gotSomethingElseOtherThan1011) {
      numberOf1011s++;  //Increase the number of times we encountered 1010 at the encoder
      //lms("Found 1011 #");
      //lmsln(numberOf1011s);
      gotSomethingElseOtherThan1011 = false;
    } else if (getRawEncoderValue() != 1010) {
      gotSomethingElseOtherThan1011 = true;
    }
    delay(20);
  }

  //Update current state
  currentState = _4HI;

  digitalWrite(RELAY_POSITIVE, HIGH);  //Stop motor
}

/******************************************************************

   Ardiono main loop

 *****************************************************************/
void loop() {
  //We have calibrated in setup to get the motor to the 2WD position

  //Start polling the dash switch every 500ms

  requestedState = getRequestedState();  //Check what the dash switch says

  if (requestedState != currentState)  //We need to move motor
  {
    if (requestedState == _2WD)  //We requested 2WD and we are not at 2WD
      find2WD();

    else if (requestedState == _4LOW)  //We requested 4LO and we are not at 4LO
      find4LO();

    else if (requestedState == _4HI && currentState == _2WD)  //From 2WD to 4HI
      find4HIFrom2WD();

    else if (requestedState == _4HI && currentState == _4LOW)  //From 4LO to 4HI
      find4HIFrom4LO();

    else if (requestedState == _4LOW && currentState == _2WD)  //From 2WD to 4LO, we can't do that one shot so let's send it to 4HI first
      find4HIFrom2WD();
  }

  delay(500);  //500ms delay before next poll
}
