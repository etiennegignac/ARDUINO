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


#define LOG 1  //All the verbose in the serial console
#define DEBUG_PIN_STATUS_ON_INTERRUPT 1
#define DEBUG_REQUESTED_STATE_PINS 0
#define DEBUG_CURRENT_STATE_PINS 1

// Little debug trick to strip the code of debug statement at compile time
#if LOG == 1
#define lms(x) Serial.print(x)
#define lmsln(x) Serial.println(x)

#else
#define lms(x)
#define lmsln(x)
#endif

/*
//Pinout (FOR WEMOS D1 MINI)
#define ENCODER_1 2 //Encoder bit 1 / connector pin 5 (purple/yellow)
#define ENCODER_2 0 //Encoder pin 2 / connector pin 3 (brown/white)
#define ENCODER_3 4 //Encoder pin 3 / connector pin 6 (white)
#define ENCODER_4 5 //Encoder pin 4 / connector pin 1 (orange/white)

#define RELAY_POSITIVE 15 //pin that controls the relay that moves the motor to positive traction (orange motor wire)
#define RELAY_NEGATIVE 16 //pin that controls the relay that moves the motor to negative traction (yellow motor wire)
*/

//Pinout (FOR ARDUINO UNO)
#define ENCODER_1 2   //Encoder bit 1 / connector pin 5 (purple/yellow)
#define ENCODER_2 3   //Encoder pin 2 / connector pin 3 (brown/white)
#define ENCODER_3 18  //Encoder pin 3 / connector pin 6 (white)
#define ENCODER_4 19  //Encoder pin 4 / connector pin 1 (orange/white)

#define RELAY_POSITIVE 4  //pin that controls the relay that moves the motor to positive traction (orange motor wire)
#define RELAY_NEGATIVE 5  //pin that controls the relay that moves the motor to negative traction (yellow motor wire)


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
int nominalState = IMPOSSIBLE;  //There's a bit of fudge factor around 4HI, we need another variable to keep track of state without reading encoder around 4HI.
int requestedState = _2WD;      //Based on dash switch position, initialize as 2WD for calibration
int directionOfMovement = STOPPED;
bool fetching4HI = false;    //We need to track when our requested state is 4HI because of some extra fudge factor
bool isCalibrating = true;   //true when doing initial or later calibration
bool isMotorMoving = false;  //true when motor is moving
bool firstStep4HI = false;   //First step to get to 4HI
bool secondStep4HI = false;   //Second step to get to 4HI
bool thirdStep4HI = false;    //Third step to get to 4HI, when going POSITIVE only

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

  //Each encoder pin needs an interrupt so we can detect any change
  //attachInterrupt(digitalPinToInterrupt(ENCODER_1), intDetected, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_2), intDetected, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_3), intDetected, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_4), intDetected, CHANGE);

  //Get our current state as part of init
  getCurrentState();

#if LOG == 1
  lmsln("-----------------  Setup complete  -----------------");
#endif
}

/******************************************************************

   Interrupt routine when encode state changes

 *****************************************************************/
void intDetected() {
  //Stop interrupts while the function runs
  noInterrupts();


  //Update what our currentstate is
  getCurrentState();
  //delay(1000);
  if (currentState == requestedState)  // we are at the right spot
  {
    //lmsln("STATE REACHED");
    //Stop everything
    isMotorMoving = false;

    firstStep4HI = false;
    secondStep4HI = false;
    thirdStep4HI = false;

    //Save nominal state, now that we have reached it
    //nominalState = requestedState;

    //Turn off both control relays
    digitalWrite(RELAY_POSITIVE, HIGH);  //For relay board, HIGH is OFF
    digitalWrite(RELAY_NEGATIVE, HIGH);  //For relay board, HIGH is OFF
  }

  // If we are not there, the motor just keeps going until next interrupt
  interrupts();  //Interrupts back on

  //If we are passed our goal
  /*if(currentState = 1 && requestedState == 2 && directionOfMovement == NEGATIVE)
  {
    //Stop everything
    isMotorMoving = false;

    //Turn off both control relays
    digitalWrite(RELAY_POSITIVE, HIGH); //For relay board, HIGH is OFF
    digitalWrite(RELAY_NEGATIVE, HIGH); //For relay board, HIGH is OFF

    lmsln("PASSED 4HI, now at 4LO");
  }*/
}

/******************************************************************

   Function: getCurrentState()

   Description: return the current state as an int.  Reads the
   encoder pins and return a preset value corresponding to the state
   of the transfer case.

   Updates global variable currentState

 *****************************************************************/
void getCurrentState() {
  bool pin1 = digitalRead(ENCODER_1);
  bool pin2 = digitalRead(ENCODER_2);
  bool pin3 = digitalRead(ENCODER_3);
  bool pin4 = digitalRead(ENCODER_4);


#if DEBUG_CURRENT_STATE_PINS == 1
  lms("Current state encoder: ");
  lms(pin1);
  lms(pin2);
  lms(pin3);
  lmsln(pin4);

#endif

  //Reads digital pins and returns the state
  if (pin1 && !pin2 && !pin3 && !pin4)
    currentState = TRANSITION;  //1000

  else if (pin1 && pin2 && pin3 && !pin4)
    currentState = _2WD;  //1110

  else if (pin1 && !pin2 && pin3 && !pin4)  //1010
  {
    //If directionOfMovement == POSITIVE, we were at 2WD going for 4HI, there will be 4 steps: get to 1011, get to 1010, get to 1011, get to 1010
    if (directionOfMovement == POSITIVE)
    {
      if(firstStep4HI) //We have passed first step but not second yet so this is it (first or repeat)
      {
        lmsln("POSITIVE SECOND STEP: 1010");
        secondStep4HI = true;
        currentState = TRANSITION;
      }

      else if(firstStep4HI && secondStep4HI && thirdStep4HI) //We went through first, second and third steps, this is the final one
      {
        lmsln("POSITIVE FOURTH STEP: 1010");
        currentState = _4HI;
      }

      else //Other catch all
        currentState = TRANSITION;
    }

    else if (directionOfMovement == NEGATIVE)//If directionOfMovement == NEGATIVE, we were at 4LO going for 4HI, there will be 2 steps: get to 1011, get to 1010
    {
      //We passed through first step already, therefore this is it
      if(firstStep4HI)
        currentState = _4HI;

      else 
        currentState = TRANSITION;
    }  

    
    /*
    1110 2WD MAX --> CALIBRATION GETS US HERE WITH which1010AreWeOn = 0
    1111
    1011
    1010 --> which1010AreWeOn == 1 
    1011
    1010 4HI <----- THIS IS THE ONE THAT WILL MAKE THE PLUG FIT (which1010AreWeOn == 2)
    1011
    1001
    1000
    1001
    1101 4LO
    1100 4LO max
    */
  }

  else if (pin1 && pin2 && !pin3 && pin4) {
    if (isMotorMoving)
      currentState = TRANSITION;  //1101
    else
      currentState = _4LOW;  //To make up for fudge factor
  }

  else if (pin1 && !pin2 && pin3 && pin4) { //1011

    //If directionOfMovement == POSITIVE, we were at 2WD going for 4HI, there will be 4 steps: get to 1011, get to 1010, get to 1011, get to 1010
    if (directionOfMovement == POSITIVE)
    {
      if(!firstStep4HI) //We haven't passed first step so this is it
      {
        lmsln("POSITIVE FIRST STEP: 1011");
        firstStep4HI = true;
        currentState = TRANSITION;
      }
 
      else if(firstStep4HI && secondStep4HI) //We passed first and second step, this must be third (first or repeat)
      {
        lmsln("POSITIVE THIRD STEP: 1011");
        thirdStep4HI = true;
        currentState = TRANSITION;
      }

      else
        currentState = TRANSITION;
      
    }
  }

  else if (pin1 && pin2 && pin3 && pin4) {
    if (isMotorMoving)
      currentState = TRANSITION;  //1111
    else
      currentState = _2WD;  //To make up for fudge factor
  }

  else if (pin1 && !pin2 && !pin3 && pin4)
    currentState = TRANSITION;  //1001

  else if (pin1 && pin2 && !pin3 && !pin4)
    currentState = _4LOW;  //1100

  //else if (pin1 && pin2 && !pin3 && pin4)
  //currentState = _4LOW;//1101

  else {
    //lmsln("Current State read IMPOSSIBLE!");
    currentState = IMPOSSIBLE;
  }

  /*
#if DEBUG_CURRENT_STATE_PINS == 1
  lms("Which means state is:");
  lmsln(currentState);
#endif*/

delay(500);
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

   Function: moveMotor():  

   Description: called after we figured out that we are not at the
   right place and know which way we need to go.  Turn relays on and
   let the interrupts figure it out.

 *****************************************************************/
void moveMotor() {
  if (isMotorMoving)  //If we are already moving,
    return;

  //If we weren't moving, we are now...
  isMotorMoving = true;

  if (directionOfMovement == POSITIVE) {
    //Turn on POSITIVE relay
#if LOG == 1
    //lmsln("Turning on motor positive");
#endif
    digitalWrite(RELAY_POSITIVE, LOW);   //For the relay board, LOW is ON
    digitalWrite(RELAY_NEGATIVE, HIGH);  //Make sure other one is OFF
  }

  else if (directionOfMovement == NEGATIVE) {
    //turn on NEGATIVE relay
#if LOG == 1
    //lmsln("Turning on motor negative");
#endif
    digitalWrite(RELAY_NEGATIVE, LOW);   //For the relay board, LOW is ON
    digitalWrite(RELAY_POSITIVE, HIGH);  //Make sure other one is OFF
  }
}



/******************************************************************

   Ardiono main loop

 *****************************************************************/
void loop() {

  while (isCalibrating) {
    lms("INITIAL CALIBRATION...");

    if (currentState != _2WD) {
      lmsln("Starting calibration");
      directionOfMovement = NEGATIVE;  //2WD position is all the way towards negative
      requestedState = _2WD;
      moveMotor();
    }

    while (currentState != _2WD)  //If we are not already at 2WD
    {
      delay(1000);  //Needed during debug phase because of serial console, might as well leave it in.
      lmsln("WAIT.");
    }

    lmsln("completed.");

    //We are done calibrating
    isCalibrating = false;
  }

  while (true)  //New main loop after calibration
  {
    // See what the dash switch says
    getRequestedState();

    //If we are not at the right State (and we are not on our way there (motor not moving))
    if (currentState != requestedState && currentState != TRANSITION && requestedState != IMPOSSIBLE && !isMotorMoving) {

      //We need to change something
#if LOG == 1
      lmsln("NEW STATE REQUESTED");
      lms("Current state: ");
      lmsln(currentState);
      lms("Request state: ");
      lmsln(requestedState);
#endif

      //Set direction of movement based on where are are and where we want to go (assuming we are not in-between at boot...)
      if (requestedState > currentState)
        directionOfMovement = POSITIVE;

      else
        directionOfMovement = NEGATIVE;

      //See if we are going for 4HI
      if (requestedState == _4HI) {
        fetching4HI = true;

        //If directionOfMovement == POSITIVE, we were at 2WD going for 4HI, there will be 3 steps: get to 1010, get to 1011, get to 1010
        //If directionOfMovement == NEGATIVE, we were at 4LO going for 4HI, there will be 2 steps: get to 1011, get to 1010
      }


      //Move the motor!
      moveMotor();

      delay(1000);
    }
    delay(1000);
  }
}
