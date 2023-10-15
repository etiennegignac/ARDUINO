/*
   Implementation of Electric Shift On the Fly (ESOF) NV273F transfer case out of a 2000 Ford F350


   Author: Etienne Gignac-Bouchard, etiennegignac@hotmail.com
   Date: 20 Nov 21


   Hardware:  Arduino
              Relay board (at least 2 relays on it)
              2 x automotive relays


 *********************************************************************
   Inputs/outputs (Check #define section below for pinout)
 *********************************************************************

   Arduino INPUTS:

              DIGITAL
              Encoder pin 1
              Encorer pin 2
              Encoder pin 3
              Encoder pin 4

   Arduino OUTPUTS:

              Output to increase traction (+) i.e. go from 2WD to 4HI to 4LOW
              Output to decrease traction (-) i.e. go from 4LOW to 4HI to 2WD
   TODO: new version to add input from dash switch
*/


#define LOG 1 //All the verbose in the serial console
#define DEBUG_PIN_STATUS_ON_INTERRUPT 0

// Little debug trick to strip the code of debug statement at compile time
#if LOG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)

#else
#define debug(x)
#define debugln(x)
#endif

//Pinout
#define ENCODER_1 1
#define ENCODER_2 2
#define ENCODER_3 3
#define ENCODER_4 4

#define RELAY_POSITIVE 5 //pin that controls the relay that moves the motor to positive traction
#define RELAY_NEGATIVE 0 //pin that controls the relay that moves the motor to negative traction

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
int currentState = IMPOSSIBLE;
int requestedState = IMPOSSIBLE;
int directionOfMovement = 0;
bool isMotorMoving = false;


/******************************************************************

   Arduino setup

 *****************************************************************/
void setup()
{
  delay(1000);//Seems to be necessary for the MKR1010 to play nice with the console
#if LOG == 1
  Serial.begin(115200);
  delay(1000);//Seems to be necessary for the MKR1010 to play nice with the console
  debugln("Setting up...");
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
  attachInterrupt(digitalPinToInterrupt(1), intDetected, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), intDetected, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), intDetected, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), intDetected, CHANGE);

  //We just booted up, read current state and requested state
  currentState = getCurrentState();
  requestedState = getRequestedState();

#if LOG == 1

  debug("Current state: ");
  debugln(getCurrentState());

  debug("Requested state: ");
  debugln(getRequestedState());

#if DEBUG_PIN_STATUS_ON_INTERRUPT == 1
  intDetected();
#endif

  debugln("Setup complete");
#endif
}

/******************************************************************

   Interrupt routine when encode state changes

 *****************************************************************/
void intDetected()
{
  //First thing we do is turn off motor to re-evaluate where we are
  //stopMotor();

  //Update what our currentstate is
  currentState = getCurrentState();

#if LOG == 1
  debug("Interrupted current state is: ");
  debugln(currentState);
  debug("Requested state is: ");
  debugln(requestedState);
#endif
  //  */

#if DEBUG_PIN_STATUS_ON_INTERRUPT == 1
  debugln("Pin 1:");
  debugln(digitalRead(1));

  debugln("Pin 2:");
  debugln(digitalRead(2));

  debugln("Pin 3:");
  debugln(digitalRead(3));

  debugln("Pin 4:");
  debugln(digitalRead(4));
#endif


  if(currentState != requestedState) // we are not at thre right spot
  {
    //Keep going (don't touch direction of rotation)
    moveMotor();
  }

  else //we are here, stop motor
  {
    stopMotor();
  }

  //Set direction of movement based on where are are and where we want to go
  /*if (requestedState > currentState)
  {
    directionOfMovement = POSITIVE;
    moveMotor();
  }
  else if (requestedState < currentState)
  {
    directionOfMovement = NEGATIVE;
    moveMotor();
  }
  else //we are at the right state
  {
    directionOfMovement = STOPPED;
    //currentState = requestedState;
  }*/
}

/******************************************************************

   getCurrentState() return the current state as an int
   TRANSITION = 0
   _2WD = 1
   _4HI = 2
   _4LOW = 3
   IMPOSSIBLE = 999 (eveything else)

 *****************************************************************/
int getCurrentState()
{
  //Reads digital pins and returns the state
  if (digitalRead(1) == 1 && digitalRead(2) == 0 && digitalRead(3) == 0 && digitalRead(4) == 0)
  {
    return TRANSITION; //1000
  }

  else if (digitalRead(1) == 0 && digitalRead(2) == 0 && digitalRead(3) == 1 && digitalRead(4) == 0)
  {
    return _2WD; //0010
  }
  else if (digitalRead(1) == 0 && digitalRead(2) == 0 && digitalRead(3) == 1 && digitalRead(4) == 1)
  {
    return _2WD;//0011, this is is with the motor at the very end of its travel
  }
  else if (digitalRead(1) == 0 && digitalRead(2) == 1 && digitalRead(3) == 1 && digitalRead(4) == 1)
  {
    return _2WD;//0111, this is is with the motor at the very end of its travel
  }

  else if (digitalRead(1) == 1 && digitalRead(2) == 0 && digitalRead(3) == 1 && digitalRead(4) == 0)
  {
    return _4HI; //1010
  }
  else if (digitalRead(1) == 1 && digitalRead(2) == 0 && digitalRead(3) == 1 && digitalRead(4) == 1)
  {
    return _4HI; //1011
  }
  else if (digitalRead(1) == 1 && digitalRead(2) == 0 && digitalRead(3) == 0 && digitalRead(4) == 1)
  {
    return _4HI; //1001
  }

  else if (digitalRead(1) == 1 && digitalRead(2) == 1 && digitalRead(3) == 0 && digitalRead(4) == 0)
  {
    return _4LOW; //1100
  }
  else if (digitalRead(1) == 1 && digitalRead(2) == 1 && digitalRead(3) == 0 && digitalRead(4) == 1)
  {
    return _4LOW; //1101
  }

  else
  {
    return 999; //impossible state
  }

}



/******************************************************************

   getRequestedState() returns the requested state based on dash switch position
   TRANSITION = 0
   _2WD = 1
   _4HI = 2
   _4LOW = 3
   IMPOSSIBLE = 999 (eveything else)

 *****************************************************************/
int getRequestedState()
{
  //TODO: Implement this to read the dash switch value, probably based on interrupts that call this

  //Temporarily: ask an int from the console...no error checking implemented

  if (Serial.available())
  {
    int incomingByte = Serial.read();
    requestedState = incomingByte - '0';

    //Flush rest of buffer
    while (Serial.read() >= 0);

    return requestedState;
  }

  else
    return currentState;
}


/******************************************************************

   moveMotor(): called after we figured out that we are not at the right place and know which way we need to go

 *****************************************************************/
void moveMotor()
{
  //We are moving...
  isMotorMoving = true;
  
  if (directionOfMovement == POSITIVE)
  {
    //Turn on POSITIVE relay
#if LOG == 1
    debugln("Turning on motor positive");
#endif
    digitalWrite(RELAY_POSITIVE, LOW); //For the relay board, LOW is ON
  }
  else if (directionOfMovement == NEGATIVE)
  {
    //turn on NEGATIVE relay
#if LOG == 1
    debugln("Turning on motor negative");
#endif
    digitalWrite(RELAY_NEGATIVE, LOW); //For the relay board, LOW is ON
  }
}

/******************************************************************

   stopMotor(): stops everything regardless of which way we are going

 *****************************************************************/
void stopMotor()
{
#if LOG == 1
  debugln("Stopping motor");
#endif

  //Stoppping
  isMotorMoving = false;

  digitalWrite(RELAY_POSITIVE, HIGH); //For relay board, HIGH is OFF
  digitalWrite(RELAY_NEGATIVE, HIGH); //For relay board, HIGH is OFF
}



/******************************************************************

   Ardiono main loop

 *****************************************************************/
void setNewRequestedStateNoSwitch()
{
  requestedState++; //go from 2WD to 4HI to 4 LOW

  if(requestedState > 3) // if we are past 4LOW, go back to 2WD
  {
    requestedState = 1;
  }
  
}

/******************************************************************

   Ardiono main loop

 *****************************************************************/
void loop()
{
  //while(1); //Pause everything for debug purposes
#if LOG == 1
  /*debug("Current state is ");

    switch(currentState)
    {
    case TRANSITION:
    debugln("Transitioning...");
    break;

    case _2WD:
    debugln("2WD");
    break;

    case _4HI:
    debugln("4HI");
    break;

    case _4LOW:
    debugln("4LOW");
    break;

    case IMPOSSIBLE:
    debugln("Impossible.");
    break;
    }*/
#endif

  if (currentState == requestedState) //Temporary while I don't have the switch
  {
    //delay a bit
    delay(10000);

    setNewRequestedStateNoSwitch(); // Set new requested state

  }

  //If we are not at the right State (and we are not on our way there (motor not moving))
  else if (currentState != requestedState && currentState != IMPOSSIBLE && requestedState != IMPOSSIBLE && !isMotorMoving)
  {
    //We need to change something
#if LOG == 1
    debugln("New state requested");
    debug("Current state: ");
    debugln(currentState);
    debug("Request state: ");
    debugln(requestedState);
#endif

    //Set direction of movement based on where are are and where we want to go
    if (requestedState > currentState)
    {
      directionOfMovement = POSITIVE;
    }
    else
    {
      directionOfMovement = NEGATIVE;
    }
    //currentState = requestedState;
    moveMotor();
  }
}
