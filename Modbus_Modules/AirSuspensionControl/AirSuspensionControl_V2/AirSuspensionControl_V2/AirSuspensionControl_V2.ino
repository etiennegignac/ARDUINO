

/*
   NOTE : If we are using the RS485 shield on top of the MKR1010wifi, we can't use the MKR's pins A4, A5, 13 and 14.
   NOTE2: Turns out we will be using a Mega2560

   INPUTS:

     ANALOG:  Supply tank pressure
              Front-left (FL) pressure
              Front-right (FR) pressure
              Rear-right (RR) pressure
              Rear-left (RL) pressure

     OUTPUTS: FL intake solenoid
              FL exhaust solenoid
              FR intake solenoid
              FR exhaust solenoid
              RR intake solenoid
              RR exhaust solenoid
              RL intake solenoid
              RL exhaust solenoid

     MPu6050:

*/
#define DEBUG_DEVELOP   1 //1 = enable console ouput while developing or debugging, 0 = disable console (in service)

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//#include <MPU6050_tockn.h>
//#include <MPU6050_light.h>

//**************** MODBUS STUFF ***********************************************
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
const int numCoils = 10;
const int numDiscreteInputs = 10;
const int numHoldingRegisters = 10;
const int numInputRegisters = 10;//*/

//Yes, we are considered a server if though we are a slave...
/*
   MODBUS RCU IDs
   1 : Master (Cab Pi)
   2 : Suspension control module (this)
*/
#define MODBUS_SLAVE_ID        2
//*****************************************************************************


#define SUPPLYTANKPPIN  A0

#define FLPPIN          A1
#define FLINPIN         2
#define FLEXPIN         3

#define FRPPIN          A2
#define FRINPIN         4
#define FREXPIN         5

#define RRPPIN          A3
#define RRINPIN          6
#define RREXPIN         7

#define RLPPIN          A4
#define RLINPIN         8
#define RLEXPIN         9

#define INTERRUPT_PIN 18  // use pin 18 for interrupt from MPU6050
#define LED_PIN 13        // (Arduino is 13, Teensy is 11, Teensy++ is 6)

//Kelderman says 175 psi for rear bags... let's be conservtive
#define MAX_REAR_BAG_PRESSURE  130
#define MAX_FRONT_BAG_PRESSURE 130

//Pressures used when slamming the frame down. Adjust to make sure nothing rubs.
#define MIN_FRONT_BAG_PRESSURE  130
#define MIN_REAR_BAG_PRESSURE   20

//Pressure value for testing purposees only (must be a float)
#define TEST_PRESSURE_UP_FRONT  20
#define TEST_PRESSURE_UP_REAR   15


//Init variables
//Temporary variables for analog to digital conversion
int analogReading = 0;
float analogReadingVolts = 0.0;

//Pressure of supply tank
float supplyTankPressurePSI = 0.0;

//Pressure of front left bag
float FLPressurePSI = 0.0;

//Pressure of front right bag
float FRPressurePSI = 0.0;

//Pressure of rear right bag
float RRPressurePSI = 0.0;

//Pressure of rear left bag
float RLPressurePSI = 0.0;

//Target pressure for bags
float targetFLPressure = 0.0;
float targetFRPressure = 0.0;
float targetRRPressure = 0.0;
float targetRLPressure = 0.0;

//Variables related to the MPU6050
//Pitch (nose up is positive)
float pitch = 0.0;

//Roll (passenger side up is positive) TODO confirm
float roll = 0.0;

//single character entered in serial console to debug (for parsing)
char command;

//Delay in ms that we keep the solenoids open to creep up on our target pressures.  Start at 1000ms (1 second)
int solenoidsOpenDelay = 1000;

//Delay in ms after we closed the solenoids to let the pressure stabilize before reading them again.
int solenoidsClosedDelay = 500;

//Instance to control the MPU6050
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

bool blinkState = false; //State variable for our led (that blinks when there's activity

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}
/*
  void disableGyro()
  {
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  }

  void enableGyro()
  {
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  }*/

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Serial.print("Test");
  //Tell the board we need to use external analog reference (5V) as opposed to the internal 3.3V
  //analogReference(AR_DEFAULT);
  //analogReference(AR_EXTERNAL);

  //Analog pressure pin as read (pressure sensors)
  pinMode(SUPPLYTANKPPIN, INPUT);
  pinMode(FLPPIN, INPUT);
  pinMode(FRPPIN, INPUT);
  pinMode(RRPPIN, INPUT);
  pinMode(RLPPIN, INPUT);

  //Output pin for solenoid control
  pinMode(FLINPIN, OUTPUT);
  pinMode(FLEXPIN, OUTPUT);
  pinMode(FRINPIN, OUTPUT);
  pinMode(FREXPIN, OUTPUT);
  pinMode(RRINPIN, OUTPUT);
  pinMode(RREXPIN, OUTPUT);
  pinMode(RLINPIN, OUTPUT);
  pinMode(RLEXPIN, OUTPUT);

  //Start serial console
#ifdef DEBUG_DEVELOP
  Serial.begin(115200);
#endif

  //Reserve logic on relay board means all output pin have to start high for the relays to be open (not activated)
  closeAllSolenoids();

  //MPU code
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
#ifdef DEBUG_DEVELOP
  Serial.println(F("Initializing I2C devices..."));
#endif
  mpu.initialize();

  //Interrupt pin
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
#ifdef DEBUG_DEVELOP
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif

  // wait for ready
  // Actually, just go right ahead and proceed...
  /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    //*/

  // load and configure the DMP
#ifdef DEBUG_DEVELOP
  Serial.println(F("Initializing DMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
#ifdef DEBUG_DEVELOP
    Serial.println(F("Enabling DMP..."));
#endif
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
#ifdef DEBUG_DEVELOP
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
#endif
    //enableGyro(); //This enables the MPU6050
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef DEBUG_DEVELOP
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
#ifdef DEBUG_DEVELOP
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  /*
     MODBUS RELATED STUFF
  */
 Serial.println("Starting modbus server");
 /* if (!ModbusRTUServer.begin(42, 9600))
  {
#ifdef DEBUG_DEVELOP
    Serial.println("Failed to start Modbus RTU Server!");
#endif
    //while (1);
  }
  else
  {

    // configure coils at address 0x00
    ModbusRTUServer.configureCoils(0x00, numCoils);

    // configure discrete inputs at address 0x00
    ModbusRTUServer.configureDiscreteInputs(0x00, numDiscreteInputs);

    // configure holding registers at address 0x00
    ModbusRTUServer.configureHoldingRegisters(0x00, numHoldingRegisters);

    // configure input registers at address 0x00
    ModbusRTUServer.configureInputRegisters(0x00, numInputRegisters);
  }//*/
}

void readSupplyTankPressure()
{
  analogReading = analogRead(SUPPLYTANKPPIN); //this is an int
  analogReadingVolts = ((float)analogReading) * 5.0 / 1023.0; // 5.0 is because internal ref of 5V, 1043 is max value on 10 bits

  // Sensor reads 0.5V at 0 psi and 4.5V at 200 psi
  //Correct for sensor range (0.5-4.5)
  if (analogReadingVolts < 0.5)
  {
    analogReadingVolts = 0.5;
  }
  else if (analogReadingVolts > 4.5)
  {
    analogReadingVolts = 4.5;
  }

  //Translate to PSI, this will have to change if using a sensor with a different scale
  supplyTankPressurePSI = (analogReadingVolts - 0.5) / 0.02;
}

void readFLPressure()
{
  analogReading = analogRead(FLPPIN); //this is an int
  analogReadingVolts = ((float)analogReading) * 5.0 / 1023.0; // 5.0 is because internal ref of 5V, 1043 is max value on 10 bits

  //Correct for sensor range (0.5-4.5)
  if (analogReadingVolts < 0.5)
  {
    analogReadingVolts = 0.5;
  }
  else if (analogReadingVolts > 4.5)
  {
    analogReadingVolts = 4.5;
  }

  //Translate to PSI, this will have to change if using a sensor with a different scale
  FLPressurePSI = (analogReadingVolts - 0.5) / 0.02;
}

void readFRPressure()
{
  analogReading = analogRead(FRPPIN); //this is an int
  analogReadingVolts = ((float)analogReading) * 5.0 / 1023.0; // 5.0 is because internal ref of 5V, 1043 is max value on 10 bits

  //Correct for sensor range (0.5-4.5)
  if (analogReadingVolts < 0.5)
  {
    analogReadingVolts = 0.5;
  }
  else if (analogReadingVolts > 4.5)
  {
    analogReadingVolts = 4.5;
  }

  //Translate to PSI, this will have to change if using a sensor with a different scale
  FRPressurePSI = (analogReadingVolts - 0.5) / 0.02;
}

void readRRPressure()
{
  analogReading = analogRead(RRPPIN); //this is an int
  analogReadingVolts = ((float)analogReading) * 5.0 / 1023.0; // 5.0 is because internal ref of 5V, 1043 is max value on 10 bits

  //Correct for sensor range (0.5-4.5)
  if (analogReadingVolts < 0.5)
  {
    analogReadingVolts = 0.5;
  }
  else if (analogReadingVolts > 4.5)
  {
    analogReadingVolts = 4.5;
  }

  //Translate to PSI, this will have to change if using a sensor with a different scale
  RRPressurePSI = (analogReadingVolts - 0.5) / 0.02;
}

void readRLPressure()
{
  analogReading = analogRead(RLPPIN); //this is an int
  analogReadingVolts = ((float)analogReading) * 5.0 / 1023.0; // 5.0 is because internal ref of 5V, 1043 is max value on 10 bits

  //Correct for sensor range (0.5-4.5)
  if (analogReadingVolts < 0.5)
  {
    analogReadingVolts = 0.5;
  }
  else if (analogReadingVolts > 4.5)
  {
    analogReadingVolts = 4.5;
  }

  //Translate to PSI, this will have to change if using a sensor with a different scale
  RLPressurePSI = (analogReadingVolts - 0.5) / 0.02;
}

void readPressures()
{
  //delay(50);
  //Serial.println("Reading supply pressure");
  readSupplyTankPressure();
  //delay(50);
  //Serial.println("Reading FL pressure");
  readFLPressure();
  //delay(50);
  readFRPressure();
  //delay(50);
  readRRPressure();
  //delay(50);
  readRLPressure();
  //delay(50);
}

void slam()
{
  /*
     Get everything as low pressure as it will go
  */

  //Flag to keep track on when we are done
  bool targetReached = false;

  //Set our target pressure to the minimum pressure used to slam
  targetFLPressure = MIN_FRONT_BAG_PRESSURE;
  targetFRPressure = MIN_FRONT_BAG_PRESSURE;
  targetRRPressure = MIN_REAR_BAG_PRESSURE;
  targetRLPressure = MIN_REAR_BAG_PRESSURE;

#ifdef DEBUG_DEVELOP
  Serial.println("SLAMMING!");
#endif
  /*
    while (!targetReached)
    {
      //Update all current pressures
      readPressures();

      //Update pitch and roll values from MPU6050
      mpu.update();
      delay(100);
      pitch = mpu.getAngleX();
      roll = mpu.getAngleY();

      Serial.print("Supply tank pressure:\t");
      Serial.println(supplyTankPressurePSI);
      /*Serial.print("Pitch:\t");
      Serial.print(pitch);
      Serial.print("\t");
      Serial.print("Roll:\t");
      Serial.println(roll);//*/

  /*
    Serial.print("FL pressure: ");
    Serial.print(FLPressurePSI);
    Serial.println("  target:  ");
    Serial.println(targetFLPressure);
  */
  /*
      //FL CORNER
      if (FLPressurePSI < targetFLPressure) // Pressure is too low, we need to go up
      {
        //Serial.println("FL up");
        digitalWrite(FLINPIN, LOW);
      }
      else if (FLPressurePSI > targetFLPressure) // Pressure is too high, we need to go down
      {
        //Serial.println("FL down");
        digitalWrite(FLEXPIN, LOW);
      }

      //FR CORNER
      if (FRPressurePSI < targetFRPressure) // Pressure is too low, we need to go up
      {
        //Serial.println("FR up");
        digitalWrite(FRINPIN, LOW);
      }
      else if (FRPressurePSI > targetFRPressure) // Pressure is too high, we need to go down
      {
        //Serial.println("FR down");
        digitalWrite(FREXPIN, LOW);
      }

      //RR CORNER
      if (RRPressurePSI < targetRRPressure) // Pressure is too low, we need to go up
      {
        //Serial.println("RR up");
        digitalWrite(RRINPIN, LOW);
      }
      else if (RRPressurePSI > targetRRPressure) // Pressure is too high, we need to go down
      {
        //Serial.println("RR down");
        digitalWrite(RREXPIN, LOW);
      }

      //RL CORNER
      if (RLPressurePSI < targetRLPressure) // Pressure is too low, we need to go up
      {
        digitalWrite(RLINPIN, LOW);
      }
      else if (RLPressurePSI > targetRLPressure) // Pressure is too high, we need to go down
      {
        digitalWrite(RLEXPIN, LOW);
      }

      //Serial.println("Delaying...");
      //Leave solenoids open a set amount of time
      delay(solenoidsOpenDelay);
      //Serial.println("Delay done, closing solenoids.");

      closeAllSolenoids();
    }*/
}

void drive()
{
  //Aims for known bag pressures for driving
}

void level()
{

}

void slamAndLevel()
{

}



void loop() {

  readPressures();

  //Read requested status
  //If requested status != current status --> move to the status

  if (DEBUG_DEVELOP)
  {
    /*
      Serial.println("------------ Current pressures ------------");

      Serial.println("       Tank");
      Serial.print("      ");
      Serial.println(supplyTankPressurePSI);
      Serial.println();

      Serial.print(FLPressurePSI);
      Serial.print("          ");
      Serial.println(FRPressurePSI);
      Serial.println();

      Serial.print(RLPressurePSI);
      Serial.print("          ");
      Serial.println(RRPressurePSI);
      Serial.println();
      Serial.println();


      Serial.println("Enter command:");
      Serial.println("    1 - Deflate for 20 seconds");
      Serial.println("    2 - Inflate for 10 seconds");
      Serial.println("    3 - Inflate to test pressure");
      Serial.println("    4 - Deflate to 5 psi");
      Serial.println("    5 - Go up and down 3 times");
      //Serial.println("    d - Enter driving mode");
      //Serial.println("    s - Slam");
      Serial.println("    u - Update current pressures");
      Serial.println("    z - Test relays and solenoids");
    */
    ///*
#ifdef DEBUG_DEVELOP
    /*Serial.print("Tank: ");
    Serial.print(supplyTankPressurePSI);
    Serial.print("   ");
    Serial.print("FL: ");
    Serial.print(FLPressurePSI);
    Serial.print("   ");
    Serial.print("FR: ");
    Serial.print(FRPressurePSI);
    Serial.print("   ");
    Serial.print("RR: ");
    Serial.print(RRPressurePSI);
    Serial.print("   ");
    Serial.print("RL: ");
    Serial.print(RLPressurePSI);*/
#endif
    //*/
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef DEBUG_DEVELOP
      Serial.print("   ");
      //Serial.print("ypr\t");
      //Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("Pitch: ");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("   ");
      Serial.print("Roll: ");
      Serial.println(ypr[2] * 180 / M_PI);
#endif
#endif
    }

  }

  //Wait for a character to be entered
  /*while (Serial.available() == 0)
    {
    //Serial.print("Serial bytes in while: ");
    //Serial.println(Serial.available());
    }*/

  //Small delay required to make sure buffer is empty
  //delay(100);

  if (Serial.available() > 0) //There is something in the buffer, command for us?
  {
    command = Serial.read();
  }

  if (command == 's')
  {
    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
      //Serial.print("Exhausting: ");
      //Serial.println(t);
    }
    slam();
    command = 0; // reset command
  }

  else if (command == 'd')
  {
    Serial.println("Back to driving pressures...");
    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }

    drive();
  }

  else if (command == 'u')
  {
    Serial.println("Updating...");
    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }
  }

  else if (command == 'z')
  {
    Serial.println("Testing relays and solenoids (sequential)...");
    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }

    digitalWrite(FLINPIN, LOW);
    delay(2000);
    digitalWrite(FLINPIN, HIGH);
    digitalWrite(FLEXPIN, LOW);
    delay(2000);
    digitalWrite(FLEXPIN, HIGH);
    digitalWrite(FRINPIN, LOW);
    delay(2000);
    digitalWrite(FRINPIN, HIGH);
    digitalWrite(FREXPIN, LOW);
    delay(2000);
    digitalWrite(FREXPIN, HIGH);
    digitalWrite(RRINPIN, LOW);
    delay(2000);
    digitalWrite(RRINPIN, HIGH);
    digitalWrite(RREXPIN, LOW);
    delay(5000);
    digitalWrite(RREXPIN, HIGH);
    digitalWrite(RLINPIN, LOW);
    delay(2000);
    digitalWrite(RLINPIN, HIGH);
    digitalWrite(RLEXPIN, LOW);
    delay(5000);
    closeAllSolenoids();
    delay(2500);

  }

  //Temporary proof-of-concept, dump everything for 20 seconds
  else if (command == '1')
  {
    Serial.println("Proof-of-concept - DUMPING");
    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }

    //Temp: deflate everything for 20 seconds
    Serial.println("Opening solenoids...");
    digitalWrite(FLEXPIN, LOW);
    digitalWrite(FREXPIN, LOW);
    digitalWrite(RREXPIN, LOW);
    digitalWrite(RLEXPIN, LOW);
    Serial.println("Delay - 20 seconds...");
    delay(20000);
    Serial.println("Closing solenoids");
    closeAllSolenoids();
    Serial.println("Done");
  }

  //Temporary proof-of-concept, pump all thew way up (inflate for 10 seconds)
  else if (command == '2')
  {
    Serial.println("Proof-of-concept - INFLATING");
    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }

    //Temp: inflate everything for 10 seconds
    Serial.println("Opening solenoids...");
    digitalWrite(FLINPIN, LOW);
    digitalWrite(FRINPIN, LOW);
    Serial.println("Delay - 10 seconds...");
    delay(10000);
    Serial.println("Closing solenoids");
    closeAllSolenoids();
    Serial.println("Done");
  }

  //Temporary proof-of-concept, inflate to "TEST" psi
  else if (command == '3')
  {
    Serial.println("Proof-of-concept - Inflating to 40 psi");
    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }

    //Make what needs to go up, go up
    int openTime = 0;
    bool targetReached = false;

    while (!targetReached)
    {
      if (FLPressurePSI < TEST_PRESSURE_UP_FRONT)
      {
        digitalWrite(FLINPIN, LOW);
      }
      if (FRPressurePSI < TEST_PRESSURE_UP_FRONT)
      {
        digitalWrite(FRINPIN, LOW);
      }
      if (RRPressurePSI < TEST_PRESSURE_UP_REAR)
      {
        digitalWrite(RRINPIN, LOW);
      }
      if (RLPressurePSI < TEST_PRESSURE_UP_REAR)
      {
        digitalWrite(RLINPIN, LOW);
      }
      else if (FLPressurePSI > TEST_PRESSURE_UP_FRONT && FRPressurePSI > TEST_PRESSURE_UP_FRONT && RRPressurePSI > TEST_PRESSURE_UP_REAR && RLPressurePSI > TEST_PRESSURE_UP_REAR) // target reached
      {
        targetReached = true;
      }

      if (TEST_PRESSURE_UP_FRONT - FLPressurePSI > 10 && TEST_PRESSURE_UP_FRONT - FRPressurePSI > 10 && TEST_PRESSURE_UP_REAR - RRPressurePSI > 10 && TEST_PRESSURE_UP_REAR - RLPressurePSI > 10) //All bags are over 10 PSIs away, open solenoids for 5 seconds
      {
        openTime = 1000;
      }
      else
      {
        //At least one corner is under 10 psi away from target, shorter open time of 1 second
        openTime = 500;
      }

      delay(openTime);
      closeAllSolenoids();
      readPressures();
      Serial.print("Supply: ");
      Serial.println(supplyTankPressurePSI);
      Serial.println();

      Serial.print(FLPressurePSI);
      Serial.print("          ");
      Serial.println(FRPressurePSI);
      Serial.print(RRPressurePSI);
      Serial.print("          ");
      Serial.println(RLPressurePSI);
      Serial.println();
      delay(1000);
    }

    Serial.println("TARGET REACHED");

  }

  //Temporary proof-of-concept, deflate to 5 psi
  else if (command == '4')
  {
    Serial.println("Proof-of-concept - Deflating to 5 psi target");
    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }

    deflateToTarget(5.0, 5.0, 5.0, 5.0);
  }

  //Temporary proof-of-concept, Cycle suspension up and down
  else if (command == '5')
  {
    Serial.println("Proof-of-concept - UP AND DOWN");

    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }

    int repetitions = 0;
    while (repetitions < 3)
    {
      Serial.println("Inflating...");
      digitalWrite(FLINPIN, LOW); //Inflate FL
      digitalWrite(FRINPIN, LOW); //Inflate FR

      Serial.println("Delay 3 seconds...");
      delay(3000);
      Serial.println("Closing Solenoids...");
      closeAllSolenoids();
      Serial.println("Delay 3 seconds...");
      delay(3000);

      Serial.println("Deflating...");
      digitalWrite(FLEXPIN, LOW); //Deflate FL
      digitalWrite(FREXPIN, LOW); //Deflate FR

      Serial.println("Delay 5s...");
      delay(5000);

      Serial.println("Closing Solenoids...");
      closeAllSolenoids();
      Serial.println("Solenoids closed");
      Serial.println("Delay 1 second...");
      delay(1000);
      Serial.println("Done.");

      repetitions++;
    }
  }

  else
  {
    //Serial.println("Unknown or empty command");
  }



  /*Serial.print("Enter target psi for bags (FL, FR, RR, RL)");
    while (Serial.available() == 0)
    {}

    if (Serial.available() > 0)
    {
    targetFLPressure = Serial.parseFloat();
    targetFRPressure = Serial.parseFloat();
    targetRRPressure = Serial.parseFloat();
    targetRLPressure = Serial.parseFloat();

    //flush the rest of the serial stream
    while (Serial.available() != 0)
    {
      char t = Serial.read();
    }

    Serial.println();
    //Serial.print("Got: ");
    //Serial.println(targetFLPressure);
    //Serial.println(targetFRPressure);
    //Serial.println(targetRRPressure);
    //Serial.println(targetRLPressure);

    //if (targetFLPressure > supplyTankPressurePSI || targetFLPressure > supplyTankPressurePSI
    //    || targetFLPressure > supplyTankPressurePSI || targetFLPressure > supplyTankPressurePSI)
    if(supplyTankPressurePSI < 75)
    {
      Serial.println("LOW AIR SUPPLY PRESSURE");
    }

    else
    {
      //Temp test serial board
      if (targetFLPressure > FLPressurePSI)
      {
        digitalWrite(FLINPIN, LOW);
        delay(1000);
        digitalWrite(FLINPIN, HIGH);
      }
      else if(targetFLPressure < FLPressurePSI)
      {
        digitalWrite(FLEXPIN, LOW);
        delay(1000);
        digitalWrite(FLEXPIN, HIGH);
      }
    }
    }*/

  //delay(1000);

}

void deflateToTarget(float FLpsi, float FRpsi, float RRpsi, float RLpsi)
{
  //Make what needs to go down, go down
  int openTime = 0;
  bool targetReached = false;

  while (!targetReached)
  {
    Serial.println("looping");
    if (FLPressurePSI > FLpsi)
    {
      digitalWrite(FLEXPIN, LOW);
    }
    if (FRPressurePSI > FRpsi)
    {
      digitalWrite(FREXPIN, LOW);
    }
    if (RRPressurePSI > RRpsi)
    {
      digitalWrite(RREXPIN, LOW);
    }
    if (RLPressurePSI > RLpsi)
    {
      digitalWrite(RLEXPIN, LOW);
    }
    else if (FLPressurePSI < FLpsi && FRPressurePSI < FRpsi && RRPressurePSI < RRpsi && RLPressurePSI < RLpsi) // target reached
    {
      Serial.println("Setting target reached");
      targetReached = true;
    }

    if (FLPressurePSI - FLpsi > 10 && FRPressurePSI - FRpsi > 10 && RRPressurePSI - RRpsi > 10 && RLPressurePSI - RLpsi > 10) //All bags are over 10 PSIs away, open solenoids for longer
    {
      Serial.println("Setting openTime = 1000");
      openTime = 1000;
    }
    else
    {
      //At least one corner is under 10 psi away from target, shorter open time of 1 second
      Serial.println("Setting openTime = 500");
      openTime = 500;
    }

    delay(openTime);
    closeAllSolenoids();

    Serial.println("Reading pressures");
    readPressures();
    Serial.print("Supply: ");
    Serial.println(supplyTankPressurePSI);
    Serial.println();
    Serial.println("looping2");
    Serial.print(FLPressurePSI);
    Serial.print("          ");
    Serial.println(FRPressurePSI);
    Serial.print(RLPressurePSI);
    Serial.print("          ");
    Serial.println(RRPressurePSI);
    Serial.println();
    delay(1000);
    Serial.println("looping3");
  }

  Serial.println("TARGET REACHED");
}

void closeAllSolenoids()
{
  /*
     Close all solenoids regardless of their status
  */
  //Serial.println("DEBUG: Closing all solenoids");
  digitalWrite(FLINPIN, HIGH);
  delay(10);
  digitalWrite(FLEXPIN, HIGH);
  delay(10);
  digitalWrite(FRINPIN, HIGH);
  delay(10);
  digitalWrite(FREXPIN, HIGH);
  delay(10);
  digitalWrite(RRINPIN, HIGH);
  delay(10);
  digitalWrite(RREXPIN, HIGH);
  delay(10);
  digitalWrite(RLINPIN, HIGH);
  delay(10);
  digitalWrite(RLEXPIN, HIGH);
  delay(10);
  //Serial.println("DEBUG: Done closing solenoids");
  //Small delay to let the pressure stabilize
  delay(solenoidsClosedDelay);
  //Serial.println("DEBUG: Exiting closeAllSolenoids() function");

  //return;
}
