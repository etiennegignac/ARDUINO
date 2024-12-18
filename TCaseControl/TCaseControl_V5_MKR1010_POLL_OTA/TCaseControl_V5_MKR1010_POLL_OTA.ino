

#include <SPI.h>
#include <WiFiNINA.h>
#include <utility/wifi_drv.h> //For MKR1010WIFI RGB LED 
#include <ArduinoOTA.h> //OTA
#include <ArduinoMqttClient.h> //MQTT

#define LOG 0  //All the verbose in the serial console
#define DEBUG_REQUESTED_STATE_PINS 0
#define DEBUG_CURRENT_STATE_PINS 0
#define DEBUG_RAW_ENCODER 1
#define SKIP_WIFI 0
#define SKIP_MQTT 0

// Little debug trick to strip the code of debug statement at compile time
//#if LOG == 1
#define lms(x) Serial.print(x)
#define lmsln(x) Serial.println(x)

//#else
//#define lms(x)
//#define lmsln(x)
//#endif

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

//Global variables
int currentState = IMPOSSIBLE;  //Based on transfer case motor position
int requestedState = IMPOSSIBLE;      //Based on dash switch position
int initialRequestedState = IMPOSSIBLE;         //Dash switch state when booting up

int rawEncoderValue = 0;  //Used to store bit vlues as int representation (1011, 1111, etc)
int pastEncoderValue = 0;

int directionOfMovement = STOPPED;  //Motor direction of movement
bool isMotorMoving = false;         //true when motor is moving
bool justBooted = true;            //Set to true when we just booted and haven't moved the dash switch yet

int switchPosition = 0;  //Variable where we store the value of the analog to digital converter on ADC pin from switch

#include "wifiInfo.h"

/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;  //network SSID
char pass[] = SECRET_PASS;  // network password
int status = WL_IDLE_STATUS;

#if SKIP_MQTT == 0
//MQTT
WiFiClient wiFiClient;
MqttClient mqttClient(wiFiClient);
#endif



/******************************************************************

   Arduino setup

 *****************************************************************/
void setup() {
//delay(5000);

//For MKR1010WIFI RGB LED
WiFiDrv::pinMode(25, OUTPUT); //define GREEN LED
WiFiDrv::pinMode(26, OUTPUT); //define RED LED
WiFiDrv::pinMode(27, OUTPUT); //define BLUE LED

//Put RGB red while setup
WiFiDrv::analogWrite(25, 0); //GREEN
WiFiDrv::analogWrite(26, 255);   //RED
WiFiDrv::analogWrite(27, 0);   //BLUE

#if LOG == 1

  //Initialize serial:
  Serial.begin(115200);
  
  while(!Serial); //Wait for serial port to be available

  lmsln("Debugging to serial enabled!");
  lmsln("Transfer case controller on MKR1010 WIFI");
  lmsln("Created for the Overland Fummins");
  lmsln("By Etienne Gignac-Bouchard");
  lmsln("October 2023");
#endif

#if SKIP_WIFI == 0            //Do we skip the wifi connection?
    int connectingTries = 0;  //Number of times we are trying to connect

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED && connectingTries != 3) {  //While we are not connected and it isn't our fourth try
    lms("Attempting to connect to SSID: ");
    lmsln(ssid);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    connectingTries++;
    //delay(5000);  //Give a 5sec delay before the next try
  }

  if (status == WL_CONNECTED) {  //If we are connected to wifi
    lmsln("SUCCESS!");
    // you're connected now, so print out the status:
    printWifiStatus();

#if SKIP_MQTT == 0
    // start the WiFi OTA library with internal (flash) based storage
    //ArduinoOTA.begin(WiFi.localIP(), "Arduino", "password", InternalStorage);

    //Connect to MQTT broker
    connectingTries = 0;  //Reset number of tries

    //Set our MQTT client ID
    mqttClient.setId("T-Case_Controller");

    while (!mqttClient.connect(MQTT_BROKER) && connectingTries != 3) {
      lms("Attempting to connect to MQTT broker ");
      lmsln(MQTT_BROKER);
      connectingTries++; //Increase our number of tries
      delay(1000);  //Give a 5sec delay before the next try
    }

    if (mqttClient.connected()) {
      lms("Connection to MQTT broker (");
      lms(MQTT_BROKER);
      lmsln(") successful!");
      //Since we are now connected to our MQTT broker, we can subscribe to topics
      //client.subscribe("/hello");

      //Publish status
      String LocalIP = String() + WiFi.localIP()[0] + "." + WiFi.localIP()[1] + "." + WiFi.localIP()[2] + "." + WiFi.localIP()[3];

      MQTT_Publish("/Overland_Fummins/T-Case_Controller/NetworkStatus", "Online");
      MQTT_Publish("/Overland_Fummins/T-Case_Controller/IP", LocalIP);
      MQTT_Publish("/Overland_Fummins/T-Case_Controller/SignalStrength", String(WiFi.RSSI()));

    } else
      lmsln("Can't connect to MQTT broker...aborting");
#endif //end if SKIP_MQTT
  }
#endif //end if SKIP_WIFI


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

/*
  //Start calibrating to make sure we get right values and positions
  lmsln("Calibrating...");
  MQTT_Publish("/Overland_Fummins/T-Case_Controller/Action", "Calibrating...");
  lmsln("Going to 2WD position.");
  MQTT_Publish("/Overland_Fummins/T-Case_Controller/Action", "Going to 2WD position.");


  //COMMENT OUT THIS SECTION FOR VERSION THAT CALIBRATES ON STARTUP
  //Start our recursive function
  find2WD();

  lmsln("Calibration finished.  Current position is 2WD.");
  MQTT_Publish("/Overland_Fummins/T-Case_Controller/Action", "Calibration finished.  Current position is 2WD.");
  lmsln("-----------------  Setup complete  -----------------");
  MQTT_Publish("/Overland_Fummins/T-Case_Controller/Action", "-----------------  Setup complete  -----------------");
  //*/

  //REPLACE BY THIS BLOCK OF CODE FOR VERSION THAT DOESN'T CALIBRATE ON STARTUP
  initialRequestedState = getRequestedState(); //This will set both initialState and currentState to our current state.



  //Push to MQTT
  switch (initialRequestedState)
  {
    case _2WD:
      //MQTT_Publish("/Overland_Fummins/T-Case_Controller/Initial_dash_switch_position", "2WD");
      break;
    
    case _4HI:
      //MQTT_Publish("/Overland_Fummins/T-Case_Controller/Initial_dash_switch_position", "4HI");
      break;
    
    case _4LOW:
      //MQTT_Publish("/Overland_Fummins/T-Case_Controller/Initial_dash_switch_position", "4LOW");
      break;

    default:
      //MQTT_Publish("/Overland_Fummins/T-Case_Controller/Initial_dash_switch_position", "IMPOSSIBLE");
      break;
  }
  
  //Now push the current t-case state to MQTT
  switch(getRawEncoderValue())
  {
    case ENCODER_POS_2WD:
      //MQTT_Publish("/Overland_Fummins/T-Case_Controller/Current_State", "2WD");
      currentState = _2WD;
      //GREEN
      WiFiDrv::analogWrite(25, 255); //GREEN
      WiFiDrv::analogWrite(26, 0);   //RED
      WiFiDrv::analogWrite(27, 0);   //BLUE
      break;
    
    case ENCODER_POS_4HI:
      //MQTT_Publish("/Overland_Fummins/T-Case_Controller/Current_State", "4HI");
      currentState = _4HI;
      //BLUE
      WiFiDrv::analogWrite(25, 0); //GREEN
      WiFiDrv::analogWrite(26, 0);   //RED
      WiFiDrv::analogWrite(27, 255);   //BLUE
      break;

    case ENCODER_POS_4LO:
      //MQTT_Publish("/Overland_Fummins/T-Case_Controller/Current_State", "4LOW");
      currentState = _4LOW;
      //RED
      WiFiDrv::analogWrite(25, 0); //GREEN
      WiFiDrv::analogWrite(26, 255);   //RED
      WiFiDrv::analogWrite(27, 0);   //BLUE
      break;

    default:
      //MQTT_Publish("/Overland_Fummins/T-Case_Controller/Current_State", "IMPOSSIBLE");
      currentState = IMPOSSIBLE;
      find2WD();

      //PURPLE?
      WiFiDrv::analogWrite(25, 0); //GREEN
      WiFiDrv::analogWrite(26, 255);   //RED
      WiFiDrv::analogWrite(27, 0);   //BLUE
      break;
  }

  delay(5000);//Wait 5s to see the result of the initial


//Blink RGB led 3 times to show that we completed setup
  //Turn off
  WiFiDrv::analogWrite(25, 0); //GREEN
  WiFiDrv::analogWrite(26, 0);   //RED
  WiFiDrv::analogWrite(27, 0);   //BLUE

  delay(250);

  //RED
  //WiFiDrv::analogWrite(25, 0); //GREEN
  WiFiDrv::analogWrite(26, 255);   //RED
  //WiFiDrv::analogWrite(27, 0);   //BLUE

  delay(250);

  //Turn off
  //WiFiDrv::analogWrite(25, 0); //GREEN
  WiFiDrv::analogWrite(26, 0);   //RED
  //WiFiDrv::analogWrite(27, 0);   //BLUE

  delay(250);

  //BLUE
  //WiFiDrv::analogWrite(25, 0); //GREEN
  //WiFiDrv::analogWrite(26, 0);   //RED
  WiFiDrv::analogWrite(27, 255);   //BLUE

  delay(250);

  //Turn off
  //WiFiDrv::analogWrite(25, 0); //GREEN
  //WiFiDrv::analogWrite(26, 0);   //RED
  WiFiDrv::analogWrite(27, 0);   //BLUE

  delay(250);

  //GREEN
  WiFiDrv::analogWrite(25, 255); //GREEN
  //WiFiDrv::analogWrite(26, 0);   //RED
  //WiFiDrv::analogWrite(27, 0);   //BLUE

  delay(1000);

  //Turn off
  WiFiDrv::analogWrite(25, 0); //GREEN
  //WiFiDrv::analogWrite(26, 0);   //RED
  //WiFiDrv::analogWrite(27, 0);   //BLUE

  
}

#if SKIP_MQTT == 0
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

   Function: MQTT_Publish()

   Description: Publishes message to MQTT topic

 *****************************************************************/

void MQTT_Publish(String topic, String payload) {

  if (mqttClient.connected())  //Make sure we are connected to the MQTT broker
  {
    //Log under T-Case-Controller topic
    mqttClient.beginMessage(topic);
    mqttClient.print(payload);
    mqttClient.endMessage();
  }
}



/******************************************************************

   Function: MQTT_Log()

   Description: Publishes info to MQTT topics both
                under the controller topic and the generic info topic

 *****************************************************************/
void MQTT_Info(String payload) {

  if (mqttClient.connected())  //Make sure we are connected to the MQTT broker
  {
    //Log under T-Case-Controller topic
    mqttClient.beginMessage("/T-Case-Controller/Log/Info");
    mqttClient.print(payload);
    mqttClient.endMessage();

    //and generic log topic
    mqttClient.beginMessage("/Log/Info");
    mqttClient.print(payload);
    mqttClient.endMessage();
  }
}

/******************************************************************

   Function: MQTT_Error()

   Description: Publishes info to MQTT topics both
                under the controller topic and the generic error topic

 *****************************************************************/
void MQTT_Error(String payload) {

  if (mqttClient.connected())  //Make sure we are connected to the MQTT broker
  {
    //Log under T-Case-Controller error topic
    mqttClient.beginMessage("/T-Case-Controller/Log/Error");
    mqttClient.print(payload);
    mqttClient.endMessage();

    //and generic log topic
    mqttClient.beginMessage("/Error");
    mqttClient.print(payload);
    mqttClient.endMessage();
  }
}


/******************************************************************

   Function: MQTT_Alarm()

   Description: Publishes info to MQTT topics both
                under the controller topic and the generic alarm topic
                THIS NEEDS ATTENTION IMMEDIATELY!

 *****************************************************************/
void MQTT_Alarm(String payload) {

  if (mqttClient.connected())  //Make sure we are connected to the MQTT broker
  {
    //Log under T-Case-Controller error topic
    mqttClient.beginMessage("/T-Case-Controller/Log/Alarm");
    mqttClient.print(payload);
    mqttClient.endMessage();

    //and generic log topic
    mqttClient.beginMessage("/Alarm");
    mqttClient.print(payload);
    mqttClient.endMessage();
  }
}
#endif //end f SKIP_MQTT from start of message_received


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
  lms("Encoder: ");
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

#if SKIP_MQTT == 0
  if (mqttClient.connected())  //Make sure we are connected to the MQTT broker
  {
    if(requestedState == _2WD)
      MQTT_Publish("/Overland_Fummins/T-Case_Controller/Requested_State", "2WD");

    else if (requestedState == _4HI)
      MQTT_Publish("/Overland_Fummins/T-Case_Controller/Requested_State", "4HI");

    else if (requestedState == _4LOW)
      MQTT_Publish("/Overland_Fummins/T-Case_Controller/Requested_State", "4LO");

    else
      MQTT_Publish("/Overland_Fummins/T-Case_Controller/Requested_State", "IMPOSSIBLE");
  }
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
  //MQTT_Info("Looking for 2WD position going negative...");

  //Turn motor on going negative
  digitalWrite(RELAY_NEGATIVE, LOW);

  //Wait a small amount of time
  delay(200);

  //See what we have
  if (getRawEncoderValue() == ENCODER_POS_2WD) {  //We are the right position after small delay, found it.
    //lmsln("Found 2WD encoder position immediately.");
    //MQTT_Info("Found 2WD encoder position immediately.");
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
    //MQTT_Info("Found 2WD encoder position after searching.");
    digitalWrite(RELAY_NEGATIVE, HIGH);  //Stop motor
    delay(100);                          //Wait a bit for things to settle
    find2WD();                           //Call this function recursively to see if we get another ENCODER_POS_2WD left of this one
  }

  //We are at the leftmost ENCODER_POS_2WD position

#if SKIP_MQTT == 0
  //Update current state
  MQTT_Publish("/Overland_Fummins/T-Case_Controller/Current_State", "2WD");
#endif

  currentState = _2WD;
  WiFiDrv::analogWrite(26, 0);   //RED
  WiFiDrv::analogWrite(25, 255); //GREEN
  WiFiDrv::analogWrite(27, 0);   //BLUE
}


/******************************************************************

   Function: find4LO():  

   Description: Finds the leftmost encoder position corresponding
                to ENCODER_POS_2WD value defined above.  This
                function gets called recursively to find the
                leftmost position.

 *****************************************************************/
void find4LO() {
  //lms("Looking for 4LO going positive...");
  //MQTT_Info("Looking for 4LO going negative");

  //Turn motor on going positive
  digitalWrite(RELAY_POSITIVE, LOW);

  //Wait a small amount of time
  delay(200);

  //See what we have
  if (getRawEncoderValue() == ENCODER_POS_4LO)  //Looks like we were at the rightmost position already
  {
    //lmsln("Found rightmost encoder position.");
    //MQTT_Info("Found 4LO position immediately.");
    digitalWrite(RELAY_POSITIVE, HIGH);
  }

  else  //We are somewhere random
  {
    while (getRawEncoderValue() != ENCODER_POS_4LO)  //Keep going until we find a ENCODER_POS_4LO position
    {
      delay(10);
    }

    //MQTT_Info("Found 4LO position after searching");
    //Now we found a ENCODER_POS_4LO position, but is it the rightmost one?
    digitalWrite(RELAY_POSITIVE, HIGH);  //Stop motor
    delay(100);
    //find4LO();
  }

  //We are at the rightmost ENCODER_POS_4LO position

#if SKIP_MQTT == 0
  //Update current state
  MQTT_Publish("/Overland_Fummins/T-Case_Controller/Current_State", "4LO");
#endif

  currentState = _4LOW;
  WiFiDrv::analogWrite(26, 255);   //RED
  WiFiDrv::analogWrite(25, 0); //GREEN
  WiFiDrv::analogWrite(27, 0);   //BLUE
}


void goto4HI() {
  lmsln("Going to 4HI");
  //MQTT_Info("Going to 4HI");

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

    //MQTT_Info("Found 4HI position after searching.");
    //Now we found a ENCODER_POS_4HI position, supposed to be the only one?
    digitalWrite(RELAY_POSITIVE, HIGH);  //Stop motor
    delay(100);

    //Update current state
  #if SKIP_MQTT == 0
    MQTT_Publish("/Overland_Fummins/T-Case_Controller/Current_State", "4HI");
  #endif

    currentState = _4HI;
    WiFiDrv::analogWrite(26, 0);   //RED
    WiFiDrv::analogWrite(25, 0); //GREEN
    WiFiDrv::analogWrite(27, 255);   //BLUE
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

    //MQTT_Info("Found 4HI position after searching");
    //Now we found a ENCODER_POS_4HI position, supposed to be the only one?
    digitalWrite(RELAY_NEGATIVE, HIGH);  //Stop motor
    delay(100);

    //Update current state
#if SKIP_MQTT == 0
    MQTT_Publish("/Overland_Fummins/T-Case_Controller/Current_State", "4HI");
#endif

    currentState = _4HI;
    WiFiDrv::analogWrite(26, 0);   //RED
    WiFiDrv::analogWrite(25, 0); //GREEN
    WiFiDrv::analogWrite(27, 255);   //BLUE
  }
}

/******************************************************************

   Ardiono main loop

 *****************************************************************/
void loop() {
  // check for WiFi OTA updates
  ArduinoOTA.poll();

  // add your normal loop code below ...

  

  //Start polling the dash switch every sec

  requestedState = getRequestedState();  //Check what the dash switch says

  lms(requestedState);
  lms(" ");
  lms(currentState);
  lms(" ");
  lms(justBooted);
  lms(" ");
  lmsln(initialRequestedState);

  if (((requestedState != currentState) && !justBooted) || (requestedState != initialRequestedState) && justBooted)  //We need to move motor
  {
    if (requestedState == _2WD) {  //We requested 2WD and we are not at 2WD
      justBooted = false;
      find2WD();
    }

    else if (requestedState == _4LOW) {  //We requested 4LO and we are not at 4LO
      justBooted = false;
      find4LO();
    }

    else if (requestedState == _4HI) { //From 2WD to 4HI
      justBooted = false;
      goto4HI();
    }
  }

  delay(1000);  //1s delay before next poll
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
