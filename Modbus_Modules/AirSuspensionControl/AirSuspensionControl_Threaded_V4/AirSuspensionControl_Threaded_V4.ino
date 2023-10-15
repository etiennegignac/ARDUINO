/*
   Implementation of air suspension control as part of the Overland Fummins project truck


   Author: Etienne Gignac-Bouchard, etiennegignac@hotmail.com
   Date: 18 Nov 21


   Hardware:  Arduino Mega 2650
              MPU06050 gyro
              Aliexpress relay board
              eBay solenoid pack
              MAX485 module for modbus, DE and RE connected together, Vcc is 5V


  ModbusSlave Library: https://github.com/yaacov/ArduinoModbusSlave

 *********************************************************************
   Inputs/outputs (Check #define section below for pinout)
 *********************************************************************

   Mega2560 INPUTS:

              ANALOG
              Supply tank pressure sensor
              Front-Left (FL) pressure sensor
              Front-Right (FR) pressure sensor
              Rear-Right (RR) pressure sensor
              Rear-Left (RL) pressure sensor

              DIGITAL
              Interrupt pin from MPU6050

   Mega2560 OUTPUTS:

              FL intake solenoid
              FL exhaust solenoid
              FR intake solenoid
              FR exhaust solenoid
              RR intake solenoid
              RR exhaust solenoid
              RL intake solenoid
              RL exhaust solenoid

   Mega2560 COMMS:

              SDA/SCL for comms with MPU6050
*/


#define LOG 1 //All the verbose in the serial console
#define OUTPUT_PITCH_ROLL 0 //Output gyro values in the main loop
#define LED_DEBUG_LEVEL 1 //blink built-in led during areWeLevel() (1 for yes, 2 for no)
#define ENABLE_MODBUS 1 //1 to enable modbus, 0 to disable
#define DEBUG_BAG_PRESSURES 0 //Enablle to debug bag pressures during interrupt


// Little debug trick to strip the code of debug statement at compile time
#if LOG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)

#else
#define debug(x)
#define debugln(x)

#endif

/******************************************************************

   LIBRARIES

 *****************************************************************/
//For timer/interrupt
#include <arduino-timer.h>

//For MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif


//For Modbus
#include <ModbusSlave.h>




/******************************************************************

   Function Prototypes, global #define(s) and variables

 *****************************************************************/
//Function prototypes
void closeAllSolenoids();
void dmpDataReady();
void enableGyro();
void disableGyro();

//Pinout
#define SUPPLYTANKPPIN  A0  //Pressure sensor - air tank

#define FLPPIN          A1  //FL Pressure sensor Pin
#define FLINPIN         2   //FL Intake Pin
#define FLEXPIN         3   //FL Exhaust Pin

#define FRPPIN          A2  //FR Pressure sensor Pin
#define FRINPIN         4   //FR Intake Pin
#define FREXPIN         5   //FR Exhaust Pin

#define RRPPIN          A3  //RR Pressure sensor Pin
#define RRINPIN         6   //RR Intake Pin
#define RREXPIN         7   //RR Exhaust Pin

#define RLPPIN          A4  //RL Pressure sensor Pin
#define RLINPIN         8   //RL Intake Pin
#define RLEXPIN         9   //RL Exhaust Pin

#define INTERRUPT_PIN 18    //Pin used as interrups for MPU6050 interrupts

//Modbus-related
/*
   MODBUS SLAVE IDs
   Master: CabPi
   1 : Suspension control module (this)
*/
#define MODBUS_SLAVE_ID 1 //ID of this slave
#define MODBUS_BAUDRATE 19200 //Baud rate of the modbus comms
#define MODBUS_SERIAL_PORT  Serial3 //Serial port to use (for Mega we use Serial3, Serial is for console)

#define RS485_CTRL_PIN  10 //Use pin 10 connected to DE and RE to control between Tx and Rx

//Number of each type of Modbus registers
#define NUM_COILS       10
#define NUM_DISCRETE_INPUTS 10
#define NUM_INPUT_REGISTERS   10
#define NUM_HOLDING_REGISTERS 10


//Operating limits
//Kelderman says 175 psi for rear bags... let's be conservtive
#define MAX_REAR_BAG_PRESSURE  130
#define MAX_FRONT_BAG_PRESSURE 130

//Pressures used when slamming the frame down. Adjust to make sure nothing rubs.
#define MIN_FRONT_BAG_PRESSURE  130
#define MIN_REAR_BAG_PRESSURE   20

//Test pressures while the truck is being built
#define TEST_PRESSURE_FRONT 20
#define TEST_PRESSURE_REAR  15

//Maximum lean in degrees where the truck is considered level (has to be float)
#define MAX_LEAN 3.0 //3 degrees

//Global Variables
//Timer for interrupt
auto mainTimer = timer_create_default();

//Boolean tracking if we are currently trying to adjust the suspension
//THIS VARIABLE WAS REPLACED BY A MODBUS COIL (modbusCoils, see below for exact index)
//bool areWeAdjusting = false;

//Boolean to track if we are in the inflating phase or not
bool areWeInflating = false;
bool areWeDeflating = false;

//Booleans to track if each corner is at its target pressure
bool FLTargetReached = false;
bool FRTargetReached = false;
bool RRTargetReached = false;
bool RLTargetReached = false;

//Stop criteria within ? psi of target plus or minus (margin of error for suspension pressures)
int margin = 3;

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
float targetFLPressure = TEST_PRESSURE_FRONT;
float targetFRPressure = TEST_PRESSURE_FRONT;
float targetRRPressure = TEST_PRESSURE_REAR;
float targetRLPressure = TEST_PRESSURE_REAR;

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
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

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


// Modbus instance
Modbus slave(MODBUS_SERIAL_PORT, MODBUS_SLAVE_ID, RS485_CTRL_PIN);

//Modbus registers
/*
   Modbus register info

   Coils:
                    [0]: areWeAdjusting? 1 = adjusting, 0 = still
                    [1]: Set to 1 by client (this program) when asking master to re-send current suspension mode
                    [2]: Suspension adjusting by user: 0 = automatic, 1 = manual
                    [3]: Manual button adjust (FL inflate)
                    [4]: Manual button adjust (FL deflate)
                    [5]: Manual button adjust (both front inflate)
                    [6]: Manual button adjust (both front deflate)
                    [7]: Manual button adjust (FR inflate)
                    [8]: Manual button adjust (FR delfate)
                    [9]: Manual button adjust (both right inflate)
                    [10]: Manual button adjust (both right deflate)
                    [11]: Manual button adjust (RR inflate)
                    [12]: Manual button adjust (RR deflate)
                    [13]: Manual button adjust (both rear inflate)
                    [14]: Manual button adjust (both rear deflate)
                    [15]: Manual button adjust (RL inflate)
                    [16]: Manual button adjust (RL deflate)
                    [17]: Manual button adjust (both left inflate)
                    [18]: Manual button adjust (both left deflate)
                    
          ...
   InputRegisters:
                    [0]: Tank Pressure (psi)
                    [1]: NOT USED
                    [2]: FL Pressure (psi)
                    [3]: NOT USED
                    [4]: FR Pressure (psi)
                    [5]: NOT USED
                    [6]: RR Pressure (psi)
                    [7]: NOT USED
                    [8]: RL Pressure (psi)
                    [9]: NOT USED
                    [10]: Pitch

    HoldingRegisters:
                    [0]: Current suspension mode (0 = slam, 1 = drive, 2 = max, 3 = custom)
                    [1]: Requested suspension mode
                    [2]: ...
*/

int modbusCoils[19]; //Coils are 1 bit read-only from master
uint16_t modbusInputRegisters[11]; //Input registers are 16-bit registers read-only from master
uint16_t modbusHoldingRegisters[2]; //Holding registers are 16-bit read/write from master




/******************************************************************

   Arduino one-time setup

 *****************************************************************/
void setup()
{
  //Serial.begin(115200);
  //Start serial console if we are debugging
#if LOG == 1
  Serial.begin(19200);
  debugln("Setting up...");
#endif

  //Analog pressure pin as read (pressure sensors)
  pinMode(SUPPLYTANKPPIN, INPUT);
  pinMode(FLPPIN, INPUT);
  pinMode(FRPPIN, INPUT);
  pinMode(RRPPIN, INPUT);
  pinMode(RLPPIN, INPUT);

  //If we just powered on, make sure all air solenoids are closed
  //Do this before setting the pins to OUTPUT to avoid chatter
  closeAllSolenoids();
  
  //Output pin for solenoid control
  pinMode(FLINPIN, OUTPUT);
  pinMode(FLEXPIN, OUTPUT);
  pinMode(FRINPIN, OUTPUT);
  pinMode(FREXPIN, OUTPUT);
  pinMode(RRINPIN, OUTPUT);
  pinMode(RREXPIN, OUTPUT);
  pinMode(RLINPIN, OUTPUT);
  pinMode(RLEXPIN, OUTPUT);

  //Output pin for built-in led debug
//#if LED_DEBUG_LEVEL == 1
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); //Start with LED off
//#endif

  //Close all Solenoids right away
  closeAllSolenoids();

  //****************************   MPU-related  **********************************

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  //initialize MPU
#if LOG == 1
  debugln(F("Initializing I2C devices..."));
#endif

  mpu.initialize();

  //Setup interrupt pin
  pinMode(INTERRUPT_PIN, INPUT);

  //verify connection
#if LOG == 1
  debugln(F("Testing device connections..."));
  debugln(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif

  //load and configure the DMP
#if LOG == 1
  Serial.println(F("Initializing DMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  //supply your own gyro offsets here, USE OTHER SKETCH TO FIND OFFSET FOR EACH MPU6050
  //mpu.setXGyroOffset(-57);
  //mpu.setYGyroOffset(-22);
  //mpu.setZGyroOffset(-78);
  //mpu.setZAccelOffset(1133);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);
#if LOG == 1
    mpu.PrintActiveOffsets();
#endif

    //turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    //Enable Arduino interrupt detection
    enableGyro();
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    //mpuIntStatus = mpu.getIntStatus();

    //Set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    //Get expected DMP packet size for later comparison
    //packetSize = mpu.dmpGetFIFOPacketSize();

    //Turn on LED to show MPU is active
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    // mpu.dmpInitialize() == ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
#if LOG == 1
    debug(F("DMP Initialization failed (code "));
    debug(devStatus);
    debug(F(")"));
#endif
  }


  //****************************   Modbus-related  **********************************
  //Register callback function for Modbus processing
  slave.cbVector[CB_READ_COILS] = modbusReadCoils; //Function code 1
  slave.cbVector[CB_WRITE_COILS] = modbusWriteCoils; //Function Code 5 and 15
  slave.cbVector[CB_READ_DISCRETE_INPUTS] = modbusReadDiscreteInputs; //Function Code 2
  slave.cbVector[CB_READ_INPUT_REGISTERS] = modbusReadInputRegisters; //Function Code 4
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = modbusReadHoldingRegister; //Function Code 3
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = modbusWriteHoldingRegisters; //Function Code 6 and 16 (check ref)
  //slave.cbVector[CB_READ_EXCEPTION_STATUS] = ... //Function Code 7

#if ENABLE_MODBUS == 1
#if LOG == 1
  debugln("Starting Modbus server");
#endif

  //Set modbus serial and slave to baud rate
  MODBUS_SERIAL_PORT.begin(MODBUS_BAUDRATE);
  slave.begin(MODBUS_BAUDRATE);
#endif

  //****************************   Timer/interrupt-related  **********************************
  //mainTimer.every(60000, timer_interrupt_function);

  //Test value for development, shorter time interval
  //mainTimer.every(5000, timer_interrupt_function);
  mainTimer.every(750, timer_interrupt_function);

  //Start the program with a requested suspension mode of "Drive"
  modbusHoldingRegisters[1] = 1;

  //To force suspension there, start with a random value for current mode
  modbusHoldingRegisters[0] = 999;

  //Initialize with areWeAdjusting = false
  modbusCoils[0] = 0;

  //Initialize by asking client to send current suspension state to match
  modbusCoils[1] = 1;

  //Initialize suspension adjustment to automomagic
  modbusCoils[2] = 0;


#if LOG == 1
  debugln("Setup complete.");
#endif

  //debugln("Setup complete.");
}



/******************************************************************

   Arduino main loop

 *****************************************************************/
void loop()
{
  //Check to see if anything changed on the modbus
  slave.poll();

  //Advance timer
  mainTimer.tick();

  // Only do gyro stuff if it is ready
  if (dmpReady)
  {
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#if OUTPUT_PITCH_ROLL == 1
      //Serial.print("ypr\t");
      //Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("Pitch: ");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("   ");
      Serial.print("Roll: ");
      Serial.println(ypr[2] * 180 / M_PI);
#endif
    }
  }
}



/******************************************************************

   MPU6050 Interrupt routine

 *****************************************************************/
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
  mpuInterrupt = true;
}



/******************************************************************

   Timer Interrupt routine
   Gets called every time mainTimer triggers an interrupt.
   Used to trigger main workflow every ? minutes

 *****************************************************************/
bool timer_interrupt_function(void *)
{
  
#if LOG == 1
  //debugln("Interrupted for periodic check.");
  //debug("Adjustment type: ");
  //debugln(modbusCoils[2]);
#endif

  //Update current pressure (this timer interrupt is used as sampling interval
  //readPressures();

#if LOG == 1
  /*debug("Current suspension mode: ");
    debugln(modbusHoldingRegisters[0]);
    debug("New suspension mode: ");
    debugln(modbusHoldingRegisters[1]);//*/
#endif

  //If we are in manual mode
  if(modbusCoils[2] == 1)
  {
    moveSuspensionManual();
    return true;
  }

  // Check if we are at the right suspension mode (if Requested != current and if we are not driving)
  // (don't want to allow suspension change while driving)
  // Also this only happens when adjustment type is automatic (modbusCoils[2] = 0)
  else if ((modbusHoldingRegisters[1] != modbusHoldingRegisters[0]) && !areWeDriving())
  {
    switch (modbusHoldingRegisters[1]) //what's the requested mode?
    {
      case 0: //Slam

        //Make all target pressures = 0
        targetFLPressure = 15;
        targetFRPressure = 15;
        targetRRPressure = 10;
        targetRLPressure = 10;

        //Reset flags
        FLTargetReached = false;
        FRTargetReached = false;
        RRTargetReached = false;
        RLTargetReached = false;
        break;

      case 1: //Drive
        //Make target pressures 15 psi rear, 20 psi front
        targetFLPressure = 25;
        targetFRPressure = 25;
        targetRRPressure = 20;
        targetRLPressure = 20;

        //Reset flags
        FLTargetReached = false;
        FRTargetReached = false;
        RRTargetReached = false;
        RLTargetReached = false;
        break;

      case 2: //Max
        //Max target pressures all around
        targetFLPressure = 40;
        targetFRPressure = 40;
        targetRRPressure = 40;
        targetRLPressure = 40;

        //Reset flags
        FLTargetReached = false;
        FRTargetReached = false;
        RRTargetReached = false;
        RLTargetReached = false;
        break;

      case 3: //Manual (when the user inflated or deflated manually
        break;

      case 4: //Auto-leveled (when the user used autolevel)
        break;
    }

    
  }
  
  //Move suspension (this function deals with automatic adjustment only)
  moveSuspensionToPressuresConcurrent();
  return true; //true to repeat the trigger
}


/******************************************************************

   CloseAllSolenoids()
   Closes all solenoids!

 *****************************************************************/
void closeAllSolenoids()
{
#if LOG == 1
  //debugln("Closing all solenoids");
#endif
  digitalWrite(FLINPIN, HIGH);
  //delay(10);
  digitalWrite(FLEXPIN, HIGH);
  //delay(10);
  digitalWrite(FRINPIN, HIGH);
  //delay(10);
  digitalWrite(FREXPIN, HIGH);
  //delay(10);
  digitalWrite(RRINPIN, HIGH);
  //delay(10);
  digitalWrite(RREXPIN, HIGH);
  //delay(10);
  digitalWrite(RLINPIN, HIGH);
  //delay(10);
  digitalWrite(RLEXPIN, HIGH);
  //delay(10);

  //Small delay to let the pressure stabilize
  //delay(solenoidsClosedDelay);
}


/******************************************************************

   enableGyro()
   Attaches the interrupt from MPU6050 to get the gyro data
   (when we care about gyro data)

 *****************************************************************/
void enableGyro()
{
#if LOG == 1
  debugln("Gyro activated.");
#endif
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
}



/******************************************************************

   disableGyro()
   Detaches the interrupt from MPU6050 to stop getting the gyro data
   (when we don't care about gyro data)

 *****************************************************************/
void disableGyro()
{
#if LOG == 1
  debugln("Gyro deactivated.");
#endif
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
}


/******************************************************************

   readSupplyTankPressure()
   Reads air supply tank pressure from analog sensor

 *****************************************************************/
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

#if ENABLE_MODBUS == 1
  //Update modbus input register
  modbusInputRegisters[0] = (int)(supplyTankPressurePSI + 0.5); //Have to add 0.5 since typecast truncates float
  //debug("New modbus tank pressure: ");
  //debugln(modbusInputRegisters[0]);
#endif
}




/******************************************************************

   readFLPressure()
   Reads Front Left bag pressure from analog sensor

 *****************************************************************/
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

#if ENABLE_MODBUS == 1
  //Update modbus input register
  modbusInputRegisters[2] = (int)(FLPressurePSI + 0.5); //Have to add 0.5 since typecast truncates float
  //debug("New modbus FL pressure: ");
  //debugln(modbusInputRegisters[2]);
#endif
}



/******************************************************************

   readFRPressure()
   Reads Front Right bag pressure from analog sensor

 *****************************************************************/
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

#if ENABLE_MODBUS == 1
  //Update modbus input register
  modbusInputRegisters[4] = (int)(FRPressurePSI + 0.5); //Have to add 0.5 since typecast truncates float
  //debug("New modbus FR pressure: ");
  //debugln(modbusInputRegisters[4]);
#endif
}



/******************************************************************

   readRRPressure()
   Reads Rear Right bag pressure from analog sensor

 *****************************************************************/
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

#if ENABLE_MODBUS == 1
  //Update modbus input register
  modbusInputRegisters[6] = (int)(RRPressurePSI + 0.5); //Have to add 0.5 since typecast truncates float
  //debug("New modbus RR pressure: ");
  //debugln(modbusInputRegisters[6]);
#endif
}



/******************************************************************

   readRLPressure()
   Reads Rear Left bag pressure from analog sensor

 *****************************************************************/
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

#if ENABLE_MODBUS == 1
  //Update modbus input register
  modbusInputRegisters[8] = (int)(RRPressurePSI + 0.5); //Have to add 0.5 since typecast truncates float
  //debug("New modbus RR pressure: ");
  //debugln(modbusInputRegisters[8]);
#endif
}



/******************************************************************

   readPressures()
   Reads all pressures from analog sensors

 *****************************************************************/
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




/******************************************************************

   areWeDriving()
   Returns true if the truck is driving, false if it is not

 *****************************************************************/
bool areWeDriving()
{
  //TODO: implement this looking at speed sensor maybe
  return false;//force value for now
}





/******************************************************************

   areWeLevel()
   Returns true if the truck is level within margin of error, false if it is not

 *****************************************************************/
bool areWeLevel()
{
  //Enable gyro
  //enableGyro();

  //Wait 1s to make sure we catch "clean" interrupt and values
  //delay(1000);

#if LOG == 1
  debug("Based on gyro values: Pitch: ");
  debug((ypr[1] * 180 / M_PI));
  debug("    Roll: ");
  debug((ypr[2] * 180 / M_PI));
  debug(" , we ");
#endif

  if (abs((ypr[1] * 180 / M_PI)) < MAX_LEAN && abs((ypr[2] * 180 / M_PI)) < MAX_LEAN) //we are within 5 degrees or level, good enough
  {
#if LOG == 1
    debugln("ARE level!");
#endif

#if LED_DEBUG_LEVEL == 1 //one short blink for on-level
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
#endif
    //Done with gyro, disable
    //disableGyro();
    return true;
  }
  else if (abs((ypr[1] * 180 / M_PI)) >= MAX_LEAN || abs((ypr[2] * 180 / M_PI)) >= MAX_LEAN) //we are not level
  {
#if LOG == 1
    debugln("are NOT level...");
#endif

#if LED_DEBUG_LEVEL == 1 //two short blinks for not level
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
#endif
    //Done with gyro, disable
    //disableGyro();
    return false;
  }
  else
  {
#if LOG == 1
    debugln("are having an error :(");
#endif
    //Done with gyro, disable
    //disableGyro();
    return false;
  }
}



/******************************************************************

   modbusReadCoils()
   Reads a single coil

 *****************************************************************/
uint8_t modbusReadCoils(uint8_t fc, uint16_t address, uint16_t length)
{
  if (fc == 1) //This is only for function code 1
  {
    /*
      debug("Sending coil #");
      debug(address);
      debug(" --> ");
      debugln(modbusCoils[address]);//*/

    //Write coil to buffer
    slave.writeCoilToBuffer(0, modbusCoils[address]);

    //Is this the master checking if it needs to re-send suspension mode?
    if (address == 1)
    {
      //The master will send it again, we can now turn off our request...
      modbusCoils[1] = 0;
    }
  }
  else
  {
    return STATUS_ILLEGAL_FUNCTION;
  }
  return STATUS_OK;
}




/******************************************************************

   modbusWriteCoils()
   Write a single coil

 *****************************************************************/
uint8_t modbusWriteCoils(uint8_t fc, uint16_t address, uint16_t length)
{
#if LOG == 1
  //Serial.print("Got request to write single coil #");
  //Serial.println(address);
#endif

  if(fc == 5) //request to write single coil
  {
    modbusCoils[address] = slave.readCoilFromBuffer(0); //0 offset always for single coil
    return STATUS_OK;
  }

  else
  {
    return STATUS_ILLEGAL_FUNCTION;
  }

  
}



/******************************************************************

   modbusReadDiscreteInputs()
   Reads a single discrete input

 *****************************************************************/
uint8_t modbusReadDiscreteInputs(uint8_t fc, uint16_t address, uint16_t length)
{
#if LOG == 1
  //debug("Got request to read single coil #");
  //debugln(address);
#endif

  return STATUS_OK;
}



/******************************************************************

   modbusReadInputRegisters()
   Reads a single input register (read only from the client)
   This is where we store our current pressures

 *****************************************************************/
uint8_t modbusReadInputRegisters(uint8_t fc, uint16_t address, uint16_t length)
{
#if LOG == 1
  //debug("Reading input register: FC= ");
  //debug(fc);
  //debug(" address= ");
  //debugln(address);
#endif

  if (fc == 4)
  {
    //Could do some error checking here that the requested address exists in out array....maybe later

    //from ref: uint8_t writeRegisterToBuffer(int offset, uint16_t value) : write one register value into the response buffer.
    //Offset is always 0 since we write only one register at the beginning of the buffer
    slave.writeRegisterToBuffer(0, modbusInputRegisters[address]);
    return STATUS_OK;
  }
  else //Wrong function code, this shouldn't happen...
  {
    return STATUS_ILLEGAL_FUNCTION;
  }

}



/******************************************************************

   modbusReadHoldingRegister()
   Reads a single holding register by the client

 *****************************************************************/
uint8_t modbusReadHoldingRegister(uint8_t fc, uint16_t address, uint16_t length)
{
  if (fc == 3) //Function code to read holding registers
  {
    /*
    debug("reading holding register ");
    debug(address);
    debug(" with value ");
    debugln(modbusHoldingRegisters[address]);//*/

    //Put the right register in the buffer
    slave.writeRegisterToBuffer(0, modbusHoldingRegisters[address]);

    return STATUS_OK;
  }

  else
  {
    return STATUS_ILLEGAL_FUNCTION;
  }
}

/******************************************************************

   modbusWriteHoldingRegisters()
   Writes a single holding register by the client

 *****************************************************************/
uint8_t modbusWriteHoldingRegisters(uint8_t fc, uint16_t address, uint16_t length)
{
  if (fc == 6) // We care about 6 only (single register)
  {
#if LOG == 1
    //debug("Address: ");
    //debugln(address);
#endif

    //Put new requested suspension mode in the register (will be caught in the main loop)
    modbusHoldingRegisters[address] = slave.readRegisterFromBuffer(0);

#if LOG == 1
    //debug("Holding register value: ");
    //debugln(modbusHoldingRegisters[address]);
#endif

    return STATUS_OK;
  }
  else
  {
    return STATUS_ILLEGAL_FUNCTION;
  }
}




/******************************************************************

   moveSuspensionToPressuresConcurrent()
   Controls inflate/deflate of bags to match the target pressures
   The difference with this V2 function is that it opens
   the required solenoids simultaneously instead of consecutively

 *****************************************************************/
void moveSuspensionToPressuresConcurrent()
{
  //if(modbusCoils[0]) //areWeAdjusting?, close solenoids to allow time before reading
  //{
    //Start by closing everything so that pressure can stabilise
    closeAllSolenoids();
    //Update current pressures (maybe a small delay before this if necessary)
    delay(250);
  //}

  //Solenoids weren't open so just read pressures
  readPressures();



  //This function has to be executed as fast as possible as to not block the main loop too much...
#if LOG == 1
  /*debug("areWeAdjusting: ");
    debugln(areWeAdjusting);
    debug("areWeInflating: ");
    debugln(areWeInflating);
    debug("FLTargetReached: ");
    debugln(FLTargetReached);
    debug("FRTargetReached: ");
    debugln(FRTargetReached);*/
  /*
    debug(modbusCoils[0]);
    debug(areWeInflating);
    debug(areWeDeflating);
    debug(FLTargetReached);
    debugln(FRTargetReached);//*/
#endif

  //We were not previously moving, start with inflate then deflate
  if (!modbusCoils[0]) //!areWeAdjusting
  {
    //Does FL corner need inflating?
    if (FLPressurePSI < (targetFLPressure - margin))
    {
      //Inflate FL
      debug("Inflating FL: ");
      debug(FLPressurePSI);
      debug(" < ");
      debug(targetFLPressure);
      debug(" - ");
      debugln(margin);
      digitalWrite(FLINPIN, LOW);
      FLTargetReached = false; //Flag that we are not there yet in this corner
      modbusCoils[0] = 1; //Flag that we are moving (modbusCoils[0] == areWeAdjusting)
      areWeInflating = true; //Flag that we are inflating
    }

    delay(10);
    //Does FR corner need inflating?
    if (FRPressurePSI < (targetFRPressure - margin))
    {
      //Inflate FR
      debug("Inflating FR: ");
      debug(FRPressurePSI);
      debug(" < ");
      debug(targetFRPressure);
      debug(" - ");
      debugln(margin);
      digitalWrite(FRINPIN, LOW);
      FRTargetReached = false; //Flag that we are not there yet in this corner
      modbusCoils[0] = 1; //Flag that we are moving (modbusCoils[0] == areweAdjusting)
      areWeInflating = true; //Flag that we are inflating
    }
    delay(10);
    //Does RR corner need inflating?
    if (RRPressurePSI < (targetRRPressure - margin))
    {
      //Inflate RR
      debug("Inflating RR: ");
      debug(RRPressurePSI);
      debug(" < ");
      debug(targetRRPressure);
      debug(" - ");
      debugln(margin);
      digitalWrite(RRINPIN, LOW);
      RRTargetReached = false; //Flag that we are not there yet in this corner
      modbusCoils[0] = 1; //Flag that we are moving (modbusCoils[0] == areWeAdjusting)
      areWeInflating = true; //Flag that we are inflating
    }

    delay(10);
    //Does RL corner need inflating?
    if (RLPressurePSI < (targetRLPressure - margin))
    {
      //Inflate RL
      debug("Inflating RL: ");
      debug(RLPressurePSI);
      debug(" < ");
      debug(targetRLPressure);
      debug(" - ");
      debugln(margin);
      digitalWrite(RLINPIN, LOW);
      RLTargetReached = false; //Flag that we are not there yet in this corner
      modbusCoils[0] = 1; //Flag that we are moving (modbusCoils[0] == areWeAdjusting)
      areWeInflating = true; //Flag that we are inflating
    }
    delay(10);
    // DELFLATING (if we are done inflating)
    if (!areWeInflating)
    {
      //Does FL corner need deflating?
      if (FLPressurePSI > (targetFLPressure + margin))
      {
        //Deflate FL
        debug("Deflating FL: ");
        debug(FLPressurePSI);
        debug(" > ");
        debug(targetFLPressure);
        debug(" + ");
        debugln(margin);
        digitalWrite(FLEXPIN, LOW);
        FLTargetReached = false; //Flag that we are not there yet in this corner
        modbusCoils[0] = 1; //Flag that we are moving (modbusCoils[0] == areWeAdjusting)
        areWeDeflating = true; //Flag that we are deflating
      }
      delay(10);
      //Does FR corner need deflating?
      if (FRPressurePSI > (targetFRPressure + margin))
      {
        //Deflate FR
        debug("Deflating FR: ");
        debug(FRPressurePSI);
        debug(" > ");
        debug(targetFRPressure);
        debug(" + ");
        debugln(margin);
        digitalWrite(FREXPIN, LOW);
        FRTargetReached = false; //Flag that we are not there yet in this corner
        modbusCoils[0] = 1; //Flag that we are moving (modbusCoils[0] == areWeAdjusting)
        areWeDeflating = true; //Flag that we are deflating
      }
      delay(10);
      //Does RR corner need deflating?
      if (RRPressurePSI > (targetRRPressure + margin))
      {
        //Deflate RR
        debug("Deflating RR: ");
        debug(RRPressurePSI);
        debug(" > ");
        debug(targetRRPressure);
        debug(" + ");
        debugln(margin);
        digitalWrite(RREXPIN, LOW);
        RRTargetReached = false; //Flag that we are not there yet in this corner
        modbusCoils[0] = 1; //Flag that we are moving (modbusCoils[0] == areWeAdjusting)
        areWeDeflating = true; //Flag that we are deflating
      }
      delay(10);
      //Does RL corner need deflating?
      if (RLPressurePSI > (targetRLPressure + margin))
      {
        //Deflate RL
        debug("Deflating RL: ");
        debug(RLPressurePSI);
        debug(" > ");
        debug(targetRLPressure);
        debug(" + ");
        debugln(margin);
        digitalWrite(RLEXPIN, LOW);
        RLTargetReached = false; //Flag that we are not there yet in this corner
        modbusCoils[0] = 1; //Flag that we are moving (modbusCoils[0] == areWeAdjusting)
        areWeDeflating = true; //Flag that we are deflating
      }
    }

    //We checked all corners and nothing needs to happen, update current mode
    if (!modbusCoils[0]) //modbusCoils[0] == areWeAdjusting
    {
      modbusHoldingRegisters[0] = modbusHoldingRegisters[1];
    }

  }

  //We are already moving, trying to inflate
  else if (modbusCoils[0] && areWeInflating) //modbusCoils[0] == areWeAdjusting
  {
    //debugln("Adjusting and inflating");
    //Does FL still need to be inflated?
    if (FLPressurePSI < (targetFLPressure - margin)) //&& (FLPressurePSI < targetFLPressure + margin))
    {
      //Inflate FL
      debug("Re-Inflating FL: ");
      debug(FLPressurePSI);
      debug(" < ");
      debug(targetFLPressure);
      debug(" - ");
      debugln(margin);
      //Open solenoid
      digitalWrite(FLINPIN, LOW);
      //FLTargetReached = true;
    }
    else
    {
      //If it doesn't, we have reached our target for this corner
      FLTargetReached = true;
    }


    //Does FR still need to be inflated?
    if (FRPressurePSI < (targetFRPressure - margin)) //&& (FRPressurePSI < targetFRPressure + margin))
    {
      //Inflate FR
      debug("Re-Inflating FR: ");
      debug(FRPressurePSI);
      debug(" < ");
      debug(targetFRPressure);
      debug(" - ");
      debugln(margin);
      //Open solenoid
      digitalWrite(FRINPIN, LOW);
    }

    else
    {
      FRTargetReached = true;
    }

    //Does RR still need to be inflated?
    if (RRPressurePSI < (targetRRPressure - margin)) //&& (FLPressurePSI < targetFLPressure + margin))
    {
      //Inflate RR
      debug("Re-Inflating RR: ");
      debug(RRPressurePSI);
      debug(" < ");
      debug(targetRRPressure);
      debug(" - ");
      debugln(margin);
      //Open solenoid
      digitalWrite(RRINPIN, LOW);
      //FLTargetReached = true;
    }
    else
    {
      //If it doesn't, we have reached our target for this corner
      RRTargetReached = true;
    }


    //Does RL still need to be inflated?
    if (RLPressurePSI < (targetRLPressure - margin)) //&& (FRPressurePSI < targetFRPressure + margin))
    {
      //Inflate RL
      debug("Re-Inflating RL: ");
      debug(RLPressurePSI);
      debug(" < ");
      debug(targetRLPressure);
      debug(" - ");
      debugln(margin);
      //Open solenoid
      digitalWrite(RLINPIN, LOW);
    }

    else
    {
      RLTargetReached = true;
    }

    //Check to see if we are done inflating
    if (FLTargetReached && FRTargetReached && RRTargetReached && RLTargetReached)
    {
#if LOG == 1
      debug("Pressures after inflation: ");
      debug(FLPressurePSI);
      debug(" | ");
      debug(FRPressurePSI);
      debug(" | ");
      debug(RRPressurePSI);
      debug(" | ");
      debugln(RLPressurePSI);
#endif
      areWeInflating = false; //We are done inflating but we still have to check if we need to deflate
      areWeDeflating = true; //Move on to deflating
      //areWeAdjusting = false; //temp
    }

  }

  //We are already moving, trying to deflate
  else if (modbusCoils[0] && areWeDeflating) //modbusCoils[0] == areweAdjusting
  {
    //debugln("Adjusting and deflating");
    //Does FL still need to be deflated?
    if (FLPressurePSI > (targetFLPressure + margin)) //&& (FLPressurePSI < targetFLPressure + margin))
    {
      //Deflate FL
      debug("Re-Deflating FL: ");
      debug(FLPressurePSI);
      debug(" > ");
      debug(targetFLPressure);
      debug(" + ");
      debugln(margin);
      //Open solenoid
      digitalWrite(FLEXPIN, LOW);
      //FLTargetReached = true;
    }
    else
    {
      //If it doesn't, we have reached our target for this corner
      FLTargetReached = true;
    }


    //Does FR still need to be deflated?
    if (FRPressurePSI > (targetFRPressure + margin)) //&& (FRPressurePSI < targetFRPressure + margin))
    {
      //Deflate FR
      debug("Re-Deflating FR: ");
      debug(FRPressurePSI);
      debug(" > ");
      debug(targetFRPressure);
      debug(" + ");
      debugln(margin);
      //Open solenoid
      digitalWrite(FREXPIN, LOW);
    }

    else
    {
      //If it doesn't, we have reached our target for this corner
      FRTargetReached = true;
    }

    //Does RR still need to be deflated?
    if (RRPressurePSI > (targetRRPressure + margin)) //&& (FLPressurePSI < targetFLPressure + margin))
    {
      //Deflate RR
      debug("Re-Deflating RR: ");
      debug(RRPressurePSI);
      debug(" > ");
      debug(targetRRPressure);
      debug(" + ");
      debugln(margin);
      //Open solenoid
      digitalWrite(RREXPIN, LOW);
      //FLTargetReached = true;
    }
    else
    {
      //If it doesn't, we have reached our target for this corner
      RRTargetReached = true;
    }


    //Does RL still need to be deflated?
    if (RLPressurePSI > (targetRLPressure + margin)) //&& (FRPressurePSI < targetFRPressure + margin))
    {
      //Deflate RL
      debug("Re-Deflating RL: ");
      debug(RLPressurePSI);
      debug(" > ");
      debug(targetRLPressure);
      debug(" + ");
      debugln(margin);
      //Open solenoid
      digitalWrite(RLEXPIN, LOW);
    }

    else
    {
      //If it doesn't, we have reached our target for this corner
      RLTargetReached = true;
    }

    //Check to see if we are done inflating
    if (FLTargetReached && FRTargetReached && RRTargetReached && RLTargetReached)
    {
#if LOG == 1
      debug("Pressures after deflation: ");
      debug(FLPressurePSI);
      debug(" | ");
      debug(FRPressurePSI);
      debug(" | ");
      debug(RRPressurePSI);
      debug(" | ");
      debugln(RLPressurePSI);
#endif
      areWeDeflating = false; //We are done deflating
      modbusCoils[0] = 0; //We are all done (modbusCoils[0] == areWeAdjusting)
      modbusHoldingRegisters[0] = modbusHoldingRegisters[1]; // New current mode = requested mode
    }

    else
    {
      //Everything is good
      modbusHoldingRegisters[0] = modbusHoldingRegisters[1]; // New current mode = requested mode
    }
  }
}



/******************************************************************

   moveSuspensionManual()
   Controls inflate/deflate of bags based on user input
   on the touchscreen

 *****************************************************************/
void moveSuspensionManual()
{
#if LOG == 1
  //debugln("We are in manual mode");
#endif

  //Need to update pressures for the display
  readPressures();

  //Check status of each coil that represent each a button on the touchscreen
  //Coils start at index [3] for front left inflate and [18] for both left deflate

  digitalWrite(FLINPIN, !modbusCoils[3]);
  digitalWrite(FLEXPIN, !modbusCoils[4]);
    
}
