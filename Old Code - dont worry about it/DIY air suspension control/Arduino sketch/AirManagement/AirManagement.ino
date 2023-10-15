//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------- Air Management Brain -----------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Monitors and controls air supply tanks, 4 corners air suspension and 4 corners central tire inflation system (ish) - CTIish


/*  ELECTICAL CONNECTIONS

   PINOUT:

        ARDUINO                           MPU6050
        POWER SECTION - Vcc               Vcc
        POWER SECTION - GND               GND
        COMMUNICATION SECTION - D21       SCL
        COMMUNICATION SECTION - D20       SDA
                                          XDA NOT CONNECTED
                                          XCL NOT CONNECTED
                                          AD0 NOT CONNECTED
        DIGITAL SECTION - D2              INT


        ARDUINO                           OTHER CONNECTIONS

        *** MAX485 MODULE ***
        COMMUNICATION SECTION - D19 (Rx1) MAX485 - RO (Rx pin)
        COMMUNICATION SECTION - D18 (Tx1) MAX485 - DI (Tx pin)

        DIGITAL SECTION - D52 -           MAX485 - DE
        DIGITAL SECTION - D53 -           MAX485 - RE


        *** OTHER COMMS ***
        DIGITAL SECTION - D4              Input for request status (least significant bit, 0bxxxxxxx?)
        DIGITAL SECTION - D7              Input for request status (second least significant bit, 0bxxxxxx?x)

        *** PRESSURE TRANSDUCERS ***
        ANALOG SECTION - A0               Input from pressure transducer on air tank #1
        ANALOG SECTION - A1               Input from pressure transducer on air tank #2
        ANALOG SECTION - A2               Input from pressure transducer on SUSPENSION FL
        ANALOG SECTION - A3               Input from pressure transducer on SUSPENSION FR
        ANALOG SECTION - A4               Input from pressure transducer on SUSPENSION RR
        ANALOG SECTION - A5               Input from pressure transducer on SUSPENSION RL
        ANALOG SECTION - A6               Input from pressure transducer on CTIish FL
        ANALOG SECTION - A7               Input from pressure transducer on CTIish FR
        ANALOG SECTION - A8               Input from pressure transducer on CTIish RR
        ANALOG SECTION - A9               Input from pressure transducer on CTIish RL

        *** SOLENOIDS - SUSPENSION ***
        DIGITAL SECTION - D22             Output to suspension solenoid - FL - PUMP
        DIGITAL SECTION - D23             Output to suspension solenoid - FL - DUMP
        DIGITAL SECTION - D24             Output to suspension solenoid - FR - PUMP
        DIGITAL SECTION - D25             Output to suspension solenoid - FR - DUMP
        DIGITAL SECTION - D26             Output to suspension solenoid - RR - PUMP
        DIGITAL SECTION - D27             Output to suspension solenoid - RR - DUMP
        DIGITAL SECTION - D28             Output to suspension solenoid - RL - PUMP
        DIGITAL SECTION - D29             Output to suspension solenoid - RL - DUMP

        *** SOLENOIDS - CTIish ***
        DIGITAL SECTION - D30             Output to CTIish solenoid - FL - PUMP
        DIGITAL SECTION - D31             Output to CTIish solenoid - FL - DUMP
        DIGITAL SECTION - D32             Output to CTIish solenoid - FR - PUMP
        DIGITAL SECTION - D33             Output to CTIish solenoid - FR - DUMP
        DIGITAL SECTION - D34             Output to CTIish solenoid - RR - PUMP
        DIGITAL SECTION - D35             Output to CTIish solenoid - RR - DUMP
        DIGITAL SECTION - D36             Output to CTIish solenoid - RL - PUMP
        DIGITAL SECTION - D37             Output to CTIish solenoid - RL - DUMP
*/

//-------------------------------------------------------
//-------------- DEBUG DEFINEs --------------------------
//-------------------------------------------------------
#define DEBUG_SERIAL
//Define DEBUG_YPR to output yaw, pitch and roll values to serial monitor in the IDE (no yaw for truck)
#define DEBUG_YPR

//Define DEBUG_PRESSURE_READ to output pressure sensor related values to the serial monitor in the IDE
#define DEBUG_PRESSURE_READ

//-------------------------------------------------------
//-------------------------------------------------------
//-------------------------------------------------------

//Include EEPROM.h to be able to read and write status byte to EEPROM for data persistency
//#include "EEPROM.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pitch;            //                      extracted value from ypr container
float roll;             //                      extracted value from ypr container


byte storedStatus;      /*                      use 2 least significant bits to keep track of status:
                                                00: PARKED: Dump all air and level from there (as low as possible)
                                                01: DRIVING: Ride height (preset pressure in each bag)
                                                10: EXTRA CLEARANCE: Extra ground clearance (inflate all bags to max)
                                                11: Not used.  For future development */
byte requestedStatus;     //                    Used to read requested status from analog inputs

int statusAddr;         // EEPROM address where the status byte is stored

float currentSupplyPressure;  // current supply pressure read by the pressure transducer (analog voltage in)
float airbagPressures[4];     /* current air pressure in each airbag
                                 [0] = Driver front
                                 [1] = Passenger front
                                 [2] = Passenger rear
                                 [3] = Driver rear
*/


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
  //Serial.println("DMP data ready!");
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
//  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
#ifdef DEBUG_SERIAL
  Serial.begin(115200); //Was 115200
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
#endif

  //We are using Serial for debug to serial console in the IDE and Serial1 for communication to the RS485 bus
  // Serial1: Rx = pin 19, Tx = pin 18
  //Serial1.begin(115200);
  //while (!Serial1); //wait for Serial1 ready

  // initialize device
  mpu.initialize();

  // verify connection
#ifdef DEBUG_SERIAL
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //UNCOMMENT THESE 2 LINES FOR INPUT BEFORE START
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    /*********************************************
                MAX485 INPUT / OUTPUT config
     *********************************************/
    //pinMode(52, OUTPUT); // MAX485 "DE" pin
    //pinMode(53, OUTPUT); // MAX485 "RE" pin
    pinMode(0, INPUT_PULLDOWN);
    //config485AsRx(); //Init the pins for receiving data

    // Make D4 and D7 inputs for status requested (pull-up resistors mean the control has to ground the input to enable)
    //pinMode(4, INPUT_PULLUP);
    //pinMode(7, INPUT_PULLUP);

    // Make analog 0 input for supply pressure transducer


    //Build the current status variable based on status of input pins
    //TODO: check for error based on returned value (0 = bad, 1 = good)
    /*int testError = readRequestedStatus();

      if(!testError)
      Serial.println("IMPOSSIBLE BUTTON STATUS");
    */

    //Read out last status byte stored in EEPROM, we store it at address 0
    //statusAddr = 0;
    //storedStatus = EEPROM.read(statusAddr);
    Serial.println("Exiting setup");
  }


  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
  }


  attachInterrupt(digitalPinToInterrupt(0), dmpDataReady, RISING);

}

/* Function to read digital inputs and put the values in the requestedStatus variable
    requested Status is type "byte"
    Pin 4 is first bit (least significant)
    Pin 7 is second bit (second least significant)
*/
int readRequestedStatus()
{
  requestedStatus = (digitalRead(4) & 0b00000001) + ((digitalRead(7) << 1) & 0b00000010);

#ifdef DEBUG_SERIAL
  Serial.print("Requested status read from pins:\t");
  Serial.println(requestedStatus, BIN);

  Serial.print("pin 4:\t");
  Serial.println(digitalRead(4));
  Serial.print("Pin 7:\t");
  Serial.println(digitalRead(7));
#endif

  if (requestedStatus == 0b11) //both pins high at the same time, impossible status
    return 0;
  else
    return 1;
}

/*
    Function to change the outputs connected to MAX485 DE and RE to LOW to enable receive mode
    Pin 52 = DE
    Pin 53 = RE
*/
int config485AsRx()
{
  //Switch pins to LOW for receive mode
  digitalWrite(52, LOW); //DE to LOW for receive
  digitalWrite(53, LOW); //RE to LOW for receive
}

/*
    Function to change the outputs connected to MAX485 DE and RE to HIGH to enable transmit mode
    Pin 52 = DE
    Pin 53 = RE
*/
int config485AsTx()
{
  //Switch pins to HIGH for transmit mode
  digitalWrite(52, HIGH); //Init to HIGH for transmit
  digitalWrite(53, HIGH); //Init to HIGH for transmit
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  //Serial.print("Entering main loop");
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //Serial.println("Waiting for input");

    //Serial1 already open, wait forever until we get something from master
    /*if (Serial1.available()) // 5 bytes for online request packet
    {
      Serial.println("Found serial data: ");
      byte incomingByte = Serial1.read(); //Read
      //Serial.println(incomingByte, HEX);
      if (incomingByte == 0x01) //This is the start byte of the packet, process form here
      {
        //Now read the rest of the packet
        byte incomingPacket[50]; //Tweak max length based on max packet length that can be sent
        int bytesRead = Serial1.readBytesUntil(0x04, incomingPacket, 50);
        Serial.print("Read ");
        Serial.print(bytesRead);
        Serial.println(" bytes");
        Serial.println("Found start of packet");
          incomingByte = Serial1.read(); //Read another byte
          if(incomingByte == 1) //Destination ID is for us (we are ID = 1)
          {
          //Serial.println("and it's for us...");
          incomingByte = Serial1.read();
          if(incomingByte == 0x60)
          {
            //Serial.println("And it's from the master");
            incomingByte = Serial1.read();
            if(incomingByte == 0)
            {
              Serial.println("WE HAVE BEEN ASKED IF WE ARE ONLINE!!");
              incomingByte = Serial1.read();
              if(incomingByte == 0x04)
              {
                Serial.println("End of transmission");
              }
            }
          }
          }
      }
    }*/



    //THIS DOESNT GO HERE BUT OK FOR NOW
    //attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);

    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
#ifdef DEBUG_SERIAL
    Serial.println(F("FIFO overflow!"));
#endif

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  
  else if (mpuIntStatus & 0x02) 
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //Extract pitch and roll values from the array (no yaw for truck control)
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;

#ifdef DEBUG_YPR
    Serial.print("Pitch:\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print("Roll:\t");
    Serial.println(roll);
#endif
  }
}
