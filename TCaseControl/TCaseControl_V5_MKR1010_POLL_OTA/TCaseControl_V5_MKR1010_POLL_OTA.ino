

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoOTA.h>
#include <MQTT.h>

#define LOG 1  //All the verbose in the serial console
#define DEBUG_REQUESTED_STATE_PINS 0
#define DEBUG_CURRENT_STATE_PINS 0
#define DEBUG_RAW_ENCODER 0
#define SKIP_WIFI 0

// Little debug trick to strip the code of debug statement at compile time
#if LOG == 1
#define lms(x) Serial.print(x)
#define lmsln(x) Serial.println(x)

#else
#define lms(x)
#define lmsln(x)
#endif

//Pinout (FOR ARDUINO MKR1010 WIFI)
#define ENCODER_1 6  //Encoder bit 1 / connector pin 5 (purple/yellow)
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

//Global variables
int currentState = IMPOSSIBLE;  //Based on transfer case motor position
int requestedState = _2WD;      //Based on dash switch position, initialize as 2WD for calibration

int rawEncoderValue = 0;  //Used to store bit vlues as int representation (1011, 1111, etc)
int pastEncoderValue = 0;

int directionOfMovement = STOPPED;  //Motor direction of movement
bool isMotorMoving = false;         //true when motor is moving

int switchPosition = 0;  //Variable where we store the value of the analog to digital converter on ADC pin from switch

#include "wifiInfo.h"

/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;  //network SSID
char pass[] = SECRET_PASS;  // network password
int status = WL_IDLE_STATUS;

//MQTT
WiFiClient net;
MQTTClient mqtt;




/******************************************************************

   Arduino setup

 *****************************************************************/
void setup() {

#if log == 1
  //Initialize serial:
  Serial.begin(115200);
  lmsln("Debugging to serial enabled!")
    lmsln("Transfer case controller on MKR1010 WIFI");
  lmsln("Created for the Overland Fummins")
    lmsln("By Etienne Gignac-Bouchard");
  lmsln("October 2023")
#endif

#if SKIP_WIFI == 0            //Do we skip the wifi connection?
    int connectingTries = 0;  //Number of times we are trying to connect

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED && connectingTries != 4) {  //While we are not connected and it isn't our fourth try
    lms("Attempting to connect to SSID: ");
    lmsln(ssid);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    connectingTries++;
    delay(5000);  //Give a 5sec delay before the next try
  }

  if (status == WL_CONNECTED) {  //If we are connected to wifi

    // you're connected now, so print out the status:
    printWifiStatus();

    // start the WiFi OTA library with internal (flash) based storage
    ArduinoOTA.begin(WiFi.localIP(), "Arduino", "password", InternalStorage);

    //Connect to MQTT broker
    mqtt.begin(MQTT_BROKER, net);
    mqtt.onMessage(messageReceived);  //Registers callsback function on MQTT message received

    connectingTries = 0;  //Reset number of tries

    while (!mqtt.connect("Transfer Case Controller") && connectingTries != 4) {
      lms("Attempting to connect to MQTT broker ");
      lmsln(MQTT_BROKER);
      delay(5000);  //Give a 5sec delay before the next try
    }

    if (mqtt.connected()) {
      lmsln("Connection to MQTT broker successful!");
      //Since we are now connected to our MQTT broker, we can subscribe to topics
      //client.subscribe("/hello");

      //Publish status
      MQTT_Info("Transfer Case Controller ONLINE!");
      MQTT_Info("SSID: " + String(WiFi.SSID()));
      MQTT_Info("IP: " + WiFi.localIP());
      MQTT_Info("Signal strength (RSSI): " + WiFi.RSSI());


    } else
      lmsln("Can't connect to MQTT broker...aborting");
  }
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
  lmsln("Calibrating...");
  MQTT_Info("Calibrating...");
  lmsln("Going to 2WD position.");
  MQTT_Info("Going to 2WD position.");

  //Start our recursive function
  find2WD();

  lmsln("Calibration finished.  Current position is 2WD.");
  MQTT_Info("Calibration finished.  Current position is 2WD.");
  lmsln("-----------------  Setup complete  -----------------");
  MQTT_Info("-----------------  Setup complete  -----------------");
}


/******************************************************************

   Function: messageReceived()

   Description: Handler for MQTT messages incoming.

 *****************************************************************/
void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}


/******************************************************************

   Function: MQTT_Log()

   Description: Publishes info to MQTT topics both
                under the controller topic and the generic info topic

 *****************************************************************/
void MQTT_Info(String payload) {

  if (mqtt.connected())  //Make sure we are connected to the MQTT broker
  {
    //Log under T-Case-Controller topic
    mqtt.publish("/T-Case-Controller/Log/Info", payload);

    //and generic log topic
    mqtt.publish("/Log/Info", payload);
  }
}

/******************************************************************

   Function: MQTT_Error()

   Description: Publishes info to MQTT topics both
                under the controller topic and the generic error topic

 *****************************************************************/
void MQTT_Error(String payload) {

  if (mqtt.connected())  //Make sure we are connected to the MQTT broker
  {
    //Log under T-Case-Controller error topic
    mqtt.publish("/T-Case-Controller/Log/Error", payload);

    //and generic log topic
    mqtt.publish("/Error", payload);
  }
}


/******************************************************************

   Function: MQTT_Alarm()

   Description: Publishes info to MQTT topics both
                under the controller topic and the generic alarm topic
                THIS NEEDS ATTENTION IMMEDIATELY!

 *****************************************************************/
void MQTT_Alarm(String payload) {

  if (mqtt.connected())  //Make sure we are connected to the MQTT broker
  {
    //Log under T-Case-Controller error topic
    mqtt.publish("/T-Case-Controller/Log/Alarm", payload);

    //and generic log topic
    mqtt.publish("/Alarm", payload);
  }
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

#if DEBUG_RAW_ENCODER == 1
  /* lms("pin 1: ");
  lmsln(digitalRead(ENCODER_1));
  lms("pin 2: ");
  lmsln(digitalRead(ENCODER_2));
  lms("pin 3: ");
  lmsln(digitalRead(ENCODER_3));
  lms("pin 4: ");
  lmsln(digitalRead(ENCODER_4));
  lmsln("");
  lmsln("");

  */
  lmsln(rawEncoderValue);

#endif

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

       Let's find the value in-between as limits(cutoffs).  Therefore: 
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

   Function: find2WD():  

   Description: Finds the leftmost encoder position corresponding
                to ENCODER_POS_2WD value defined above.  This
                function gets called recursively to find the
                leftmost position.

 *****************************************************************/
void find2WD() {
  lms("Looking for 2WD position going negative...");
  MQTT_Info("Looking for 2WD position going negative...");

  //Turn motor on going negative
  digitalWrite(RELAY_NEGATIVE, LOW);

  //Wait a small amount of time
  delay(200);

  //See what we have
  if (getRawEncoderValue() == ENCODER_POS_2WD) {  //We are the right position after small delay, found it.
    lmsln("Found 2WD encoder position immediately.");
    MQTT_Info("Found 2WD encoder position immediately.");
    digitalWrite(RELAY_NEGATIVE, HIGH);  //Stop motor
  }

  else  //We are somewhere random
  {
    while (getRawEncoderValue() != ENCODER_POS_2WD)  //Keep going until we find a ENCODER_POS_2WD value
    {

      //lms("Not there yet.");

      //lms("Encoder: ");
      //lmsln(getRawEncoderValue());
      delay(10);
    }

    //Now we found a ENCODER_POS_2WD position, but is it the leftmost one?
    MQTT_Info("Found 2WD encoder position after searching.");
    digitalWrite(RELAY_NEGATIVE, HIGH);  //Stop motor
    delay(100);                          //Wait a bit for things to settle
    find2WD();                           //Call this function recursively to see if we get another ENCODER_POS_2WD left of this one
  }

  //We are at the leftmost ENCODER_POS_2WD position

  //Update current state
  currentState = _2WD;
}


/******************************************************************

   Function: find4LO():  

   Description: Finds the leftmost encoder position corresponding
                to ENCODER_POS_2WD value defined above.  This
                function gets called recursively to find the
                leftmost position.

 *****************************************************************/
void find4LO() {
  lms("Looking for 4LO going positive...");
  MQTT_Info("Looking for 4LO going negative");

  //Turn motor on going positive
  digitalWrite(RELAY_POSITIVE, LOW);

  //Wait a small amount of time
  delay(200);

  //See what we have
  if (getRawEncoderValue() == ENCODER_POS_4LO)  //Looks like we were at the rightmost position already
  {
    //lmsln("Found rightmost encoder position.");
    MQTT_Info("Found 4LO position immediately.");
    digitalWrite(RELAY_POSITIVE, HIGH);
  }

  else  //We are somewhere random
  {
    while (getRawEncoderValue() != ENCODER_POS_4LO)  //Keep going until we find a ENCODER_POS_4LO position
    {
      delay(10);
    }

    MQTT_Info("Found 4LO position after searching");
    //Now we found a ENCODER_POS_4LO position, but is it the rightmost one?
    digitalWrite(RELAY_POSITIVE, HIGH);  //Stop motor
    delay(100);
    //find4LO();
  }

  //We are at the rightmost ENCODER_POS_4LO position

  //Update current state
  currentState = _4LOW;
}


void goto4HI() {
  lmsln("Going to 4HI");
  MQTT_Info("Going to 4HI");

  // If we are at 2WD, we need to go positive
  if (currentState == _2WD) {

    //Turn motor on going positive
    digitalWrite(RELAY_POSITIVE, LOW);

    //Wait a small amount of time
    delay(300);

    while (getRawEncoderValue() != ENCODER_POS_4HI)  //Keep going until we find a ENCODER_POS_4HI position
    {
      delay(10);
    }

    MQTT_Info("Found 4HI position after searching.");
    //Now we found a ENCODER_POS_4HI position, supposed to be the only one?
    digitalWrite(RELAY_POSITIVE, HIGH);  //Stop motor
    delay(100);

    //Update current state
    currentState = _4HI;
  }

  else if (currentState == _4LOW) {
    //lmsln("Going to 4HI");

    //Turn motor on going negative
    digitalWrite(RELAY_NEGATIVE, LOW);

    //Wait a small amount of time
    delay(300);

    while (getRawEncoderValue() != ENCODER_POS_4HI)  //Keep going until we find a ENCODER_POS_4HI position
    {
      delay(10);
    }

    MQTT_Info("Found 4HI position after searching");
    //Now we found a ENCODER_POS_4HI position, supposed to be the only one?
    digitalWrite(RELAY_NEGATIVE, HIGH);  //Stop motor
    delay(100);

    //Update current state
    currentState = _4HI;
  }
}

/******************************************************************

   Ardiono main loop

 *****************************************************************/
void loop() {
  // check for WiFi OTA updates
  ArduinoOTA.poll();

  // add your normal loop code below ...

  //We have calibrated in setup to get the motor to the 2WD position

  //Start polling the dash switch every 50ms

  requestedState = getRequestedState();  //Check what the dash switch says

  if (requestedState != currentState)  //We need to move motor
  {
    if (requestedState == _2WD)  //We requested 2WD and we are not at 2WD
      find2WD();

    else if (requestedState == _4LOW)  //We requested 4LO and we are not at 4LO
      find4LO();

    else if (requestedState == _4HI)  //From 2WD to 4HI
      goto4HI();
  }

  delay(50);  //500ms delay before next poll
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  lms("SSID: ");
  lmsln(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  lms("IP Address: ");
  lmsln(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  lms("signal strength (RSSI):");
  lms(rssi);
  lmsln(" dBm");
}
