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


#define LOG 1 //All the verbose in the serial console
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

//Pinout (UPDATED FOR WEMOS D1 MINI)
#define ENCODER_1 2 //Encoder bit 1 / connector pin 5 (purple/yellow)
#define ENCODER_2 0 //Encoder pin 2 / connector pin 3 (brown/white)
#define ENCODER_3 4 //Encoder pin 3 / connector pin 6 (white)
#define ENCODER_4 5 //Encoder pin 4 / connector pin 1 (orange/white)

#define RELAY_POSITIVE 13 //pin that controls the relay that moves the motor to positive traction (orange motor wire)
#define RELAY_NEGATIVE 12 //pin that controls the relay that moves the motor to negative traction (yellow motor wire)

#define ANALOG_SWITCH_IN A0 //pin where we read the voltage coming back from the switch (voltage divider with other resistance)

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

/******************************************************************

  OTA related

 *****************************************************************/

//These libraries for ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "wifiInfo.h"



/*****************************************************************/


//Global variables
int currentState = IMPOSSIBLE; //Based on transfer case motor position
int requestedState = IMPOSSIBLE; //Based on dash switch position
int encoderValue = 0; //This will be a int representation of the 4 bits (1111, 1010, 1011, etc), used as a temp variable
int directionOfMovement = STOPPED;
bool isMotorMoving = false;

int switchPosition = 0; //Variable where we store the value of the analog to digital converter on ADC pin from switch

int lastPinsRead = 0;


/******************************************************************

   Arduino setup

 *****************************************************************/
void setup()
{
 
#if LOG == 1
  Serial.begin(115200);
  lmsln("Booting...");
#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int retry = 1; // Number of tries to connect
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    lms("(Try ");
    lms(retry);
    lms(" of 3)...Connection to ");
    lms(ssid);
    lmsln(" Failed!");
    retry++;

    if(retry > 3)
    {
      lms("Can't connect to ");
      lms(ssid);
      lmsln(" , working offline");
      break;
    }
    delay(5000);
  }

  if(WiFi.waitForConnectResult() == WL_CONNECTED)
  {
    lms("CONNECTED to ");
    lms(ssid);
    lms(". IP: ");
    lmsln(WiFi.localIP());
  }
    

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
  //attachInterrupt(digitalPinToInterrupt(ENCODER_3), intDetected, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_4), intDetected, CHANGE);

  //Get our current state as part of init
  //getCurrentState();

#if LOG == 1
  lmsln("-----------------  Setup complete  -----------------");
#endif

}

/******************************************************************

   Interrupt routine when encode state changes

 *****************************************************************/
IRAM_ATTR void intDetected()
{
  //Stop interrupts while the function runs
  
  //Update what our currentstate is
  getCurrentState();

  if (currentState == requestedState) // we are at the right spot
  {
    //Stop everything
    isMotorMoving = false;

    //Turn off both control relays
    digitalWrite(RELAY_POSITIVE, HIGH); //For relay board, HIGH is OFF
    digitalWrite(RELAY_NEGATIVE, HIGH); //For relay board, HIGH is OFF
  }

  // If we are not there, the motor just keeps going until next interrupt
}

/******************************************************************

   Function: getCurrentState()

   Description: return the current state as an int.  Reads the
   encoder pins and return a preset value corresponding to the state
   of the transfer case.

   Updates global variable currentState

 *****************************************************************/
IRAM_ATTR void getCurrentState()
{
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
  {
    encoderValue = 1000;
    currentState = TRANSITION;//1000
  }

  //else if (!pin1 && !pin2 && pin3 && !pin4)
    //currentState = _2WD;//0010
    
  //else if (!pin1 && !pin2 && pin3 && pin4)
    //currentState = _2WD;//0011

  else if (!pin1 && pin2 && pin3 && pin4)
  {
    encoderValue = 111;
    currentState = _2WD;//0111
  }

  else if (pin1 && !pin2 && pin3 && !pin4)
  {
    encoderValue = 1010;
    currentState = _4HI;//1010
  }
  
  else if (pin1 && !pin2 && pin3 && pin4)
  {
    encoderValue = 1011;
    currentState = TRANSITION;//1011
  }

  else if (pin1 && !pin2 && !pin3 && pin4)
  {
    encoderValue = 1001;
    currentState = TRANSITION;//1001
  }

  else if (pin1 && pin2 && !pin3 && !pin4)
  {
    encoderValue = 1100;
    currentState = _4LOW;//1100
  }

  else if (pin1 && pin2 && !pin3 && pin4)
  {
    encoderValue = 1101;
    currentState = _4LOW;//1101
  }
  
  else if (pin1 && pin2 && pin3 && pin4) // Very max of travel
  {
    lmsln("FOUND MAX TRAVEL");
    encoderValue = 1111;
    currentState = 1111;//1111
  }

  else
  {
    //lmsln("Current State read IMPOSSIBLE!");
    currentState = IMPOSSIBLE;
  }

#if DEBUG_CURRENT_STATE_PINS == 1
 // lms("Which means state is:");
 // lmsln(currentState);
#endif

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
int getRequestedState()
{
  /* With external resistor R = 470
   *  and Vref = 3.3v (from board)
     Value of analogRead is:
       2WD: 624  \
                    511 cutoff
       4HI: 397  / 
                 \
                    317 cutoff
       4LO: 236  /

       Let's find the value in-between as limits:
       2WD > 511
       511 > 4HI > 317
       10 < 4LO < 317
       IMPOSSIBLE < 0 (switched wiring probably unplugged)
  */

  //Read the position of the dash switch
  switchPosition = analogRead(ANALOG_SWITCH_IN);

  //Figure out the position based on the values calculated as limits
  if (switchPosition > 511)
    requestedState = _2WD;
  
  else if (switchPosition <= 511 && switchPosition >= 317)
    requestedState = _4HI;
  
  else if(switchPosition < 317 && switchPosition > 10)
    requestedState = _4LOW;

  else
    requestedState = IMPOSSIBLE;

#if DEBUG_REQUESTED_STATE_PINS
  lms("Requested state read from switch: ");
  lms(switchPosition);
  lms(" --> ");
  lmsln(requestedState);
#endif

  return requestedState; // 0 = TRANSITION, 1 = 2WD, 2 = 4HI, 3 = 4LO, 999 = IMPOSSIBLE
}


/******************************************************************

   Function: moveMotor():  

   Description: called after we figured out that we are not at the
   right place and know which way we need to go.  Turn relays on and
   let the interrupts figure it out.

 *****************************************************************/
void moveMotor()
{
  //If we weren't moving, we are now...
  isMotorMoving = true;

  if (directionOfMovement == POSITIVE)
  {
    //Turn on POSITIVE relay
#if LOG == 1
    lmsln("Turning on motor positive");
#endif
    digitalWrite(RELAY_POSITIVE, LOW); //For the relay board, LOW is ON
  }
  
  else if (directionOfMovement == NEGATIVE)
  {
    //turn on NEGATIVE relay
#if LOG == 1
    lmsln("Turning on motor negative");
#endif
    digitalWrite(RELAY_NEGATIVE, LOW); //For the relay board, LOW is ON
  }
}


/******************************************************************

   Function: goTo2WD():  

   Description: Moved the motor to the 2WD position based on encoder position

 *****************************************************************/
void goTo2WD()
{
  //Move a bit away from 2WD to make sure we have valid readings
  digitalWrite(RELAY_POSITIVE, LOW);
  delay(500);
  digitalWrite(RELAY_POSITIVE, HIGH);  
  
  //We just want to go negative until we reach max travel
  digitalWrite(RELAY_NEGATIVE, LOW); //move motor towards 2WD

  //If we made it all the way to 4LO, make sure we have a delay to skip the 4LO pos when coming back
  //Because it is the same as 2WD (1111)
  delay(500);

  while(currentState != 1111)
  {
    getCurrentState();
    lms("currentState = ");
    lmsln(currentState);
    delay(100);
  } //As long as we didn't reachmax travel

  //lmsln("TURNIN OFF MOTOR");
  digitalWrite(RELAY_NEGATIVE, HIGH);
}

/******************************************************************

   Function: goTo4LO():  

   Description: Moved the motor to the 4LO position based on encoder position

 *****************************************************************/
void goTo4LO()
{
  //Move a bit away from 4LO to make sure we have valid readings
  digitalWrite(RELAY_NEGATIVE, LOW);
  delay(500);
  digitalWrite(RELAY_NEGATIVE, HIGH);

  //We just want to go positive until we reach max travel
  digitalWrite(RELAY_POSITIVE, LOW); //move motor towards 4LO

  //If we made it all the way to 2WD, make sure we have a delay to skip the 2WD pos when coming back
  //Because it is the same as 4LO (1111)
  delay(500);

  while(currentState != 1111)
  {
    getCurrentState();
    lms("currentState = ");
    lmsln(currentState);
    delay(100);
  } //As long as we didn't reachmax travel

  //lmsln("TURNIN OFF MOTOR");
  digitalWrite(RELAY_NEGATIVE, HIGH);
}


/******************************************************************

   Ardiono main loop

 *****************************************************************/
void loop()
{
  /*
      The new plan: not use interrupts.  We will be polling every 100ms(?) and hopefully catch every state of the encoder.
  */

  //First we calibrate by going to the 2WD position
  goTo2WD();
  delay(1000);
  //goTo4LO();
  //delay(1000);
  //goTo2WD();





  lmsln("SLEEPING NOW...");
  delay(600000);


  /*
  
  // See what the dash switch says
  getRequestedState();

  //If we are not at the right State (and we are not on our way there (motor not moving))
  if (currentState != requestedState && currentState != IMPOSSIBLE && requestedState != IMPOSSIBLE && !isMotorMoving)
  {
    //We need to change something
#if LOG == 1
    lmsln("LET'S MOVE!!!");
    lms("Current state: ");
    lmsln(currentState);
    lms("Request state: ");
    lmsln(requestedState);
#endif

    //Set direction of movement based on where are are and where we want to go
    if (requestedState > currentState)
      directionOfMovement = POSITIVE;
      
    else
      directionOfMovement = NEGATIVE;

    //Move the motor!
    moveMotor();
  }
  else
    delay(1000);

    */
}
