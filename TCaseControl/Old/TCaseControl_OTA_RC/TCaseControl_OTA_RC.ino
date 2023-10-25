/*
   Author: Etienne Gignac-Bouchard, etiennegignac@hotmail.com
   Date: 19 Oct 22


   Implementation of Electric Shift On the Fly (ESOF) of a NV273F
   transfer case out of a 2000 Ford F350 in a 2012 Ford F550

   Hardware:  Arduino MKR 1010 wifi
              Relay board (at least 2 relays on it)
              2 x automotive relays

 *********************************************************************
   Transfer case connectors pinout
 *********************************************************************

  Encoder:
    pin 1: orange/white
    pin 2: orange
    pin 3: brown/white
    pin 4: purple/yellow
    pin 5: purple/yellow
    pin 6: white

  Motor connector
    pin 1: Yellow: positive when less traction
    pin 2: Orange: [psotive when more traction

 *********************************************************************
   Inputs/outputs (Check #define section below for pinout)
 *********************************************************************

   Arduino INPUTS:

              DIGITAL
              Encoder pin 1 (purple/yellow)
              Encorer pin 2 (brown/white)
              Encoder pin 3 (white)
              Encoder pin 4 (orange/white)

   Arduino OUTPUTS:

              Output to increase traction (+) i.e. go from 2WD to 4HI to 4LOW (to control t-case orange motor wire)
              Output to decrease traction (-) i.e. go from 4LOW to 4HI to 2WD (to control t-case yellow motor wire)

   TODO: new version to add input from dash switch

*/


#define LOG 1 //All the verbose in the serial console
#define DEBUG_PIN_STATUS_ON_INTERRUPT 1

// Little debug trick to strip the code of debug statement at compile time
#if LOG == 1
#define debug1(x) Serial.print(x)
#define debugln1(x) Serial.println(x)

#else
#define debug1(x)
#define debugln1(x)
#endif

//Pinout
#define ENCODER_1 1 //Encoder bit 1 / connector pin 4 (purple/yellow)
#define ENCODER_2 2 //Encoder pin 2 / connector pin 3 (brown/white)
#define ENCODER_3 3 //Encoder pin 3 / connector pin 6 (white)
#define ENCODER_4 4 //Encoder pin 4 / connector pin 1 (orange/white)

#define RELAY_POSITIVE 5 //pin that controls the relay that moves the motor to positive traction (orange motor wire)
#define RELAY_NEGATIVE 0 //pin that controls the relay that moves the motor to negative traction (yellow motor wire)

#define ANALOG_SWITCH_IN A1 //pin where we read the voltage coming back from the switch (voltage divider with other resistance)

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

//OTA related stuff
#define SECRET_SSID "Garage24"
#define SECRET_PASS "Potato123!"
#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoOTA.h>



//Global variables
int currentState = IMPOSSIBLE; //Based on transfer case motor position
int requestedState = IMPOSSIBLE; //Based on dash switch position
int directionOfMovement = 0;
bool isMotorMoving = false;

int switchPosition = 0; //Variable where we store the value of the analog to digital converter on ADC pin from switch

int lastPinsRead = 0;

char ssid[] = SECRET_SSID;      // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password

int status = WL_IDLE_STATUS;

/******************************************************************

   Arduino setup

 *****************************************************************/
void setup()
{
  //delay(1000);//Seems to be necessary for the MKR1010 to play nice with the console
#if LOG == 1
  Serial.begin(115200);
  //delay(1000);//Seems to be necessary for the MKR1010 to play nice with the console
  debugln1("Setting up...");
#endif

  // check for the presence of the shield: THIS SHOULD NEVER HAPPEN WITH A MKR1010 WIFI
  if (WiFi.status() == WL_NO_SHIELD)
  {
    debugln1("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to Wifi network:

  /*while ( status != WL_CONNECTED)
    {
    debug1("Attempting to connect to SSID: ");
    debugln1(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    }*/

  //Just attempt to connect once at this time, figure out trying to reconnect later
  debug1("Attempting to connect to SSID: ");
  debugln1(ssid);
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  status = WiFi.begin(ssid, pass);

  //Are we connected?
  if (status != WL_CONNECTED)
    debugln1("Can't connect to wifi.");

  else
  {
    debugln1("Connected to wifi.");
    // start the WiFi OTA library with internal (flash) based storage
    ArduinoOTA.begin(WiFi.localIP(), "TransferCaseControl", "password", InternalStorage);

    // you're connected now, so print out the status:
    printWifiStatus();
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
  attachInterrupt(digitalPinToInterrupt(1), intDetected, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), intDetected, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), intDetected, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), intDetected, CHANGE);

  //We just booted up, read current state and requested state
  currentState = getCurrentState();
  requestedState = getRequestedState();

#if LOG == 1

  debug1("Current state: ");
  debugln1(getCurrentState());

  debug1("Requested state: ");
  debugln1(getRequestedState());

#if DEBUG_PIN_STATUS_ON_INTERRUPT == 1
  intDetected();
#endif

  debugln1("Setup complete");
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
  debug1("Interrupted current state is: ");
  debugln1(currentState);
  debug1("Requested state is: ");
  debugln1(requestedState);
#endif
  //  */

#if DEBUG_PIN_STATUS_ON_INTERRUPT == 1
  debug1("Encoder: ");
  debug1(digitalRead(1));
  debug1(digitalRead(2));
  debug1(digitalRead(3));
  debugln1(digitalRead(4));
#endif


  if (currentState != requestedState) // we are not at thre right spot
  {
    //Keep going (don't touch direction of rotation)
    moveMotor();
  }

  else //we are here, stop motor
  {
    debug1("Last pin read: ");
    debugln1(lastPinsRead);
    stopMotor();
  }
}

/******************************************************************

   Function: getCurrentState()

   Description: return the current state as an int.  Reads the
   encoder pins and return a preset value corresponding to the state
   of the transfer case.

   Possible return values:
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
    lastPinsRead = 1;
    return TRANSITION; //1000
  }

  else if (digitalRead(1) == 0 && digitalRead(2) == 0 && digitalRead(3) == 1 && digitalRead(4) == 0)
  {
    lastPinsRead = 2;
    return _2WD; //0010
  }
  else if (digitalRead(1) == 0 && digitalRead(2) == 0 && digitalRead(3) == 1 && digitalRead(4) == 1)
  {
    lastPinsRead = 3;
    return _2WD;//0011, this is is with the motor at the very end of its travel
  }
  else if (digitalRead(1) == 0 && digitalRead(2) == 1 && digitalRead(3) == 1 && digitalRead(4) == 1)
  {
    lastPinsRead = 4;
    return _2WD;//0111, this is is with the motor at the very end of its travel
  }

  else if (digitalRead(1) == 1 && digitalRead(2) == 0 && digitalRead(3) == 1 && digitalRead(4) == 0)
  {
    lastPinsRead = 5;
    return _4HI; //1010
  }
  else if (digitalRead(1) == 1 && digitalRead(2) == 0 && digitalRead(3) == 1 && digitalRead(4) == 1)
  {
    lastPinsRead = 6;
    return TRANSITION;//_4HI; //1011
  }
  else if (digitalRead(1) == 1 && digitalRead(2) == 0 && digitalRead(3) == 0 && digitalRead(4) == 1)
  {
    lastPinsRead = 7;
    return TRANSITION;//_4HI; //1001
  }

  else if (digitalRead(1) == 1 && digitalRead(2) == 1 && digitalRead(3) == 0 && digitalRead(4) == 0)
  {
    lastPinsRead = 8;
    return _4LOW; //1100
  }
  else if (digitalRead(1) == 1 && digitalRead(2) == 1 && digitalRead(3) == 0 && digitalRead(4) == 1)
  {
    lastPinsRead = 9;
    return _4LOW; //1101
  }

  else
  {
    lastPinsRead = 10;
    return 999; //impossible state
  }

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
  /* With external resistor R =
     Value of analogRead is:
       2WD: 582
       4HI: 374
       4LO: 223

       Let's find the value in-between as limits:
       2WD > 478
       478 > 4HI > 299
       4LO < 299
  */

  //Read the position of the dash switch
  switchPosition = analogRead(ANALOG_SWITCH_IN);

  //Figure out the position based on the values calculated as limits
  if (switchPosition > 478)
    requestedState = _2WD;
  
  else if (switchPosition <= 478 && switchPosition >= 299)
    requestedState = _4HI;
  
  else
    requestedState = _4LOW;

  return requestedState;
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
    debugln1("Turning on motor positive");
#endif
    digitalWrite(RELAY_POSITIVE, LOW); //For the relay board, LOW is ON
  }
  
  else if (directionOfMovement == NEGATIVE)
  {
    //turn on NEGATIVE relay
#if LOG == 1
    debugln1("Turning on motor negative");
#endif
    digitalWrite(RELAY_NEGATIVE, LOW); //For the relay board, LOW is ON
  }
}

/******************************************************************

   Function: stopMotor() 
   
   Description: stops everything regardless of which way we are going

 *****************************************************************/
void stopMotor()
{
  //Stoppping
  isMotorMoving = false;

  //Turn off both control relays
  digitalWrite(RELAY_POSITIVE, HIGH); //For relay board, HIGH is OFF
  digitalWrite(RELAY_NEGATIVE, HIGH); //For relay board, HIGH is OFF
}



/******************************************************************



 *****************************************************************/
void setNewRequestedStateNoSwitch()
{
  requestedState++; //go from 2WD to 4HI to 4 LOW

  if (requestedState > 3) // if we are past 4LOW, go back to 2WD
  {
    requestedState = 1;
  }

}

/******************************************************************

   printWifiStatus(): prints the wifi info to serial

 *****************************************************************/
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  debug1("SSID: ");
  debugln1(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  debug1("IP Address: ");
  debugln1(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  debug1("signal strength (RSSI):");
  debug1(rssi);
  debugln1(" dBm");
}

/******************************************************************

   Ardiono main loop

 *****************************************************************/
void loop()
{
  // check for WiFi OTA updates
  ArduinoOTA.poll();

  // See what the dash switch says
  getRequestedState();

  //If we are not at the right State (and we are not on our way there (motor not moving))
  if (currentState != requestedState && currentState != IMPOSSIBLE && requestedState != IMPOSSIBLE && !isMotorMoving)
  {
    //We need to change something
#if LOG == 1
    debugln1("LET'S MOVE!!!");
    debug1("Current state: ");
    debugln1(currentState);
    debug1("Request state: ");
    debugln1(requestedState);
#endif

    //Set direction of movement based on where are are and where we want to go
    if (requestedState > currentState)
      directionOfMovement = POSITIVE;
      
    else
      directionOfMovement = NEGATIVE;

    //Move the motor!
    moveMotor();
  }
}
