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
bool areWeAdjusting = false;

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
                    [0]: Suspension working? 1 = adjusting, 0 = still
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
                    [1]: ...
                    [2]: ...
*/

int modbusCoils[10]; //Coils are 1 bit read-only from master
uint16_t modbusInputRegisters[14]; //Input registers are 16-bit registers read-only from master
uint16_t modbusHoldingRegisters[10]; //Holding registers are 16-bit read/write from master




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
#if LED_DEBUG_LEVEL == 1
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); //Start with LED off
#endif

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
  slave.cbVector[CB_WRITE_COILS] = modbusWriteCoils; //Function Code 1
  slave.cbVector[CB_READ_DISCRETE_INPUTS] = modbusReadDiscreteInputs; //Function Code 2
  slave.cbVector[CB_READ_INPUT_REGISTERS] = modbusReadInputRegisters; //Function Code 4
  //slave.cbVector[CB_READ_HOLDING_REGISTERS] = ... //Function Code 3
  //slave.cbVector[CB_READ_WRITE_COILS] = ... //Function Code 5 and 15 (check ref)
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
  mainTimer.every(1000, timer_interrupt_function);

  //If we just powered on, make sure all air solenoids are closed
  closeAllSolenoids();


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



  // if programming failed, don't try to do anything
  if (!dmpReady) return;

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

  //Advance timer
  mainTimer.tick();
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
#endif

  //Update current pressure (this timer interrupt is used as sampling interval
  readPressures();
  //moveSuspensionToPressures();
  moveSuspensionToPressuresConcurrent();

  // Check if we are at the right suspension mode
  /*if (0)//(modbusHoldingRegisters[1] != modbusHoldingRegisters[0]) //Requested != current
  {
    switch (modbusHoldingRegisters[1]) //what's the requested mode?
    {
      case 0: //Slam
        //Make all target pressures = 0
        targetFLPressure = 0;
        targetFRPressure = 0;
        targetRRPressure = 0;
        targetRLPressure = 0;
        break;

      case 1: //Drive
        //Make target pressures 15 psi rear, 20 psi front
        targetFLPressure = 20;
        targetFRPressure = 20;
        targetRRPressure = 15;
        targetRLPressure = 15;
        break;

      case 2: //Max
        //Make target pressures 50 all around
        targetFLPressure = 50;
        targetFRPressure = 50;
        targetRRPressure = 50;
        targetRLPressure = 50;
        break;

      case 3: //Manual (when the user inflated or deflated manually
        break;

      case 4: //Auto-leveled (when the user used autolevel)
        break;
    }
  }*/
  
  return true;
  //END TEMPORARY ----------------------------------------------------------


  // CODE WILL NEVER GET HERE

  //Change global variable since we have started adjusting
  areWeAdjusting = true;
  while (areWeAdjusting)
  {
    if (FLPressurePSI < targetFLPressure)
    {
      digitalWrite(FLINPIN, LOW);
    }
    if (FRPressurePSI < targetFRPressure)
    {
      digitalWrite(FRINPIN, LOW);
    }
    if (RRPressurePSI < targetRRPressure)
    {
      digitalWrite(RRINPIN, LOW);
    }
    if (RLPressurePSI < targetRLPressure)
    {
      digitalWrite(RLINPIN, LOW);
    }
    else

      if (targetFLPressure - FLPressurePSI > 10 && targetFRPressure - FRPressurePSI > 10 && targetRRPressure - RRPressurePSI > 10 && targetRLPressure - RLPressurePSI > 10) //All bags are over 10 PSIs away, open solenoids for 5 seconds
      {
        //openTime = 1000;
      }
      else
      {
        //At least one corner is under 10 psi away from target, shorter open time of 1 second
        //openTime = 500;
      }
  }


  //If we are driving
  if (areWeDriving())
  {
    //Update all pressure to current
    readPressures();

    //Check for extreme low pressure only
    if (FLPressurePSI < 5.0 || FRPressurePSI < 5.0 || RRPressurePSI < 0 || RLPressurePSI < 5.0)
    {
      //Do some sort of warning here
    }
    else
    {
      return true; //Pressure is still above 5 psi, do nothing
    }
  }

  else //We are not driving, check that truck is level
  {
    if (areWeLevel())
    {
      return true;//True so next timer gets interrupted too
    }
    else //We are not level, do something about it
    {
      return true;//True so next timer gets interrupted too
    }
  }

  return true; //true to repeat the trigger, false to stop it
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

   keepPressures()
   Temporary function while the truck is still in the garage being built
   Keeps all pressures at a set amount

 *****************************************************************/
void keepPressures()
{
  // TEMPORARY CODE WHILE THE TRUCK IS STILL IN THE GARAGE
  if (!areWeAdjusting) //We are not already trying to adjust, check if we have to....
  {
    if (0) //(FLPressurePSI > TEST_PRESSURE_FRONT && FRPressurePSI > TEST_PRESSURE_FRONT && RRPressurePSI > TEST_PRESSURE_REAR && RLPressurePSI > TEST_PRESSURE_REAR) // target reached
    {
      closeAllSolenoids();
      //We are on target, stop adjusting
      areWeAdjusting = false;
      //Nothing else to do
      return;
    }
    else //Need to do some adjusting to airbag pressure
    {
      //Change flag since we started adjusting
      //areWeAdjusting = true;

      //Tweak corners one by one
      //Front left
      //while (FLPressurePSI < TEST_PRESSURE_FRONT)
      if (FLPressurePSI < targetFLPressure)
      {
#if DEBUG_BAG_PRESSURES == 1
        debug("Need to inflate Front Left (");
        debug(FLPressurePSI);
        debug(" / ");
        debug(targetFLPressure);
        debugln(") psi");
#endif
        //Put air in for 500ms and reevaluate
        digitalWrite(FLINPIN, LOW);
        delay(500);
        digitalWrite(FLINPIN, HIGH);
        //delay(10000); // delay for pressure to stabilize and to slow down this loop
        //readFLPressure();
        //delay(100);
      }



#if DEBUG_BAG_PRESSURES == 1
      debug("Final Front Left pressure: ");
      debug(FLPressurePSI);
      debugln(" psi");
#endif



      //Front right
      //while (FRPressurePSI < TEST_PRESSURE_FRONT)
      if (FRPressurePSI < targetFRPressure)
      {
#if DEBUG_BAG_PRESSURES == 1
        debug("Need to inflate Front Right (");
        debug(FRPressurePSI);
        debug(" / ");
        debug(targetFRPressure);
        debugln(") psi");
#endif
        //Put air in for 500ms and reevaluate
        digitalWrite(FRINPIN, LOW);
        delay(500);
        digitalWrite(FRINPIN, HIGH);
        //delay(5000); // delay for pressure to stabilize and to slow down this loop
        //readFRPressure();
        //delay(100);
      }


#if DEBUG_BAG_PRESSURES == 1
      debug("Final Front Right pressure: ");
      debug(FRPressurePSI);
      debugln(" psi");
#endif

      //Rear right
      //while (RRPressurePSI < TEST_PRESSURE_REAR)
      if (RRPressurePSI < targetRRPressure)
      {
#if DEBUG_BAG_PRESSURES == 1
        debug("Need to inflate Rear Right (");
        debug(RRPressurePSI);
        debug(" / ");
        debug(targetRRPressure);
        debugln(") psi");
#endif
        //Put air in for 500ms and reevaluate
        digitalWrite(RRINPIN, LOW);
        delay(500);
        digitalWrite(RRINPIN, HIGH);
        //delay(5000); // delay for pressure to stabilize and to slow down this loop
        //readRRPressure();
        //delay(100);
      }
      else
        digitalWrite(RRINPIN, HIGH);

#if DEBUG_BAG_PRESSURES == 1
      debug("Final Rear Right pressure: ");
      debug(RRPressurePSI);
      debugln(" psi");
#endif


      //Rear left
      //while (RLPressurePSI < TEST_PRESSURE_REAR)
      if (RLPressurePSI < targetRLPressure)
      {
#if DEBUG_BAG_PRESSURES == 1
        debug("Need to inflate Rear Left (");
        debug(RLPressurePSI);
        debug(" / ");
        debug(targetRLPressure);
        debugln(") psi");
#endif
        //Put air in for 500ms and reevaluate
        digitalWrite(RLINPIN, LOW);
        delay(500);
        digitalWrite(RLINPIN, HIGH);
        //delay(5000); // delay for pressure to stabilize and to slow down this loop
        //readRLPressure();
        //delay(100);
      }

#if DEBUG_BAG_PRESSURES == 1
      debug("Final Rear Left pressure: ");
      debug(RLPressurePSI);
      debugln(" psi");
#endif

      //Change the flag since we have stopped adjusting
      //areWeAdjusting = false;

#if DEBUG_BAG_PRESSURES == 1
      debugln("Pressures reached");
#endif
    }
  }
}



/******************************************************************

   modbusWriteCoils()
   Write a single coil

 *****************************************************************/
uint8_t modbusWriteCoils(uint8_t fc, uint16_t address, uint16_t length)
{
#if LOG == 1
  Serial.print("Got request to write single coil #");
  Serial.println(address);
#endif

  return STATUS_OK;
}



/******************************************************************

   modbusReadDiscreteInputs()
   Reads a single discrete input

 *****************************************************************/
uint8_t modbusReadDiscreteInputs(uint8_t fc, uint16_t address, uint16_t length)
{
#if LOG == 1
  debug("Got request to read single coil #");
  debugln(address);
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

   modbusWriteHoldingRegisters()
   Writes a single holding register by the client

 *****************************************************************/
uint8_t modbusWriteHoldingRegisters(uint8_t fc, uint16_t address, uint16_t length)
{
  if (fc == 6) // We care about 6 only (single register)
  {
    //Put new requested suspension mode in the register (will be caught in the main loop)
    modbusHoldingRegisters[address] = slave.readRegisterFromBuffer(0);
  }
  else
  {
    return STATUS_ILLEGAL_FUNCTION;
  }
}



/******************************************************************

   moveSuspensionToPressures()
   Controls inflate/deflate of bags to match the target pressures

 *****************************************************************/
void moveSuspensionToPressures()
{
  
  //Inflate bags that need to be inflated first
  int inflateTimeFront = 250; //Time to keep the inflate solenoids open for front bags
  int deflateTimeFront = 100; //Time to keep the deflate solenoids open for front bags
  int inflateTimeRear = 250; //Time to keep the inflate solenoids open for rear bags
  int deflateTimeRear = 100; //Time to keep the deflate solenoids open for rear bags
  
  int margin = 3; //Stop criteria within 3 psi of target plus or minus
  bool targetReached = false;

  //Inflate what needs to go up first
  if (FLPressurePSI < (targetFLPressure - margin))
  {
    //Inflate FL
    digitalWrite(FLINPIN, LOW);
    delay(inflateTimeFront);
    digitalWrite(FLINPIN, HIGH);
  }

  if (FRPressurePSI < (targetFRPressure - margin))
  {
    //Inflate FR
    digitalWrite(FRINPIN, LOW);
    delay(inflateTimeFront);
    digitalWrite(FRINPIN, HIGH);
  }

  if (RRPressurePSI < (targetRRPressure - margin))
  {
    //Inflate RR
    digitalWrite(RRINPIN, LOW);
    delay(inflateTimeRear);
    digitalWrite(RRINPIN, HIGH);
  }

  if (RLPressurePSI < (targetRLPressure - margin))
  {
    //Inflate RL
    digitalWrite(RLINPIN, LOW);
    delay(inflateTimeRear);
    digitalWrite(RLINPIN, HIGH);
  }

  //then deflate what is out of range
  if(FLPressurePSI > targetFLPressure + margin)
  {
    //debug("Deflating FL: ");
    //debug(FLPressurePSI);
    //debug(" > ");
    //debug(targetFLPressure);
    //debug(" + ");
    //debugln(margin);
    digitalWrite(FLEXPIN, LOW);
    delay(deflateTimeFront);
    digitalWrite(FLEXPIN, HIGH);
  }

  if(FRPressurePSI > targetFRPressure + margin)
  {
    //debug("Deflating FL: ");
    //debug(FLPressurePSI);
    //debug(" > ");
    //debug(targetFLPressure);
    //debug(" + ");
    //debugln(margin);
    digitalWrite(FREXPIN, LOW);
    delay(deflateTimeFront);
    digitalWrite(FREXPIN, HIGH);
  }

  if(RRPressurePSI > targetRRPressure + margin)
  {
    //debug("Deflating FL: ");
    //debug(FLPressurePSI);
    //debug(" > ");
    //debug(targetFLPressure);
    //debug(" + ");
    //debugln(margin);
    digitalWrite(RREXPIN, LOW);
    delay(deflateTimeFront);
    digitalWrite(RREXPIN, HIGH);
  }

  if(RLPressurePSI > targetRLPressure + margin)
  {
    //debug("Deflating FL: ");
    //debug(FLPressurePSI);
    //debug(" > ");
    //debug(targetFLPressure);
    //debug(" + ");
    //debugln(margin);
    digitalWrite(RLEXPIN, LOW);
    delay(deflateTimeFront);
    digitalWrite(RLEXPIN, HIGH);
  }

  //Delay to leave the solenoids open
  //delay(inflateTimeFront);
  //Close everything
  //closeAllSolenoids();
  //Delay to stabilize pressures before reading
  //delay(100);
  //Update the pressures
  //readPressures();
  //delay(250);
}

/******************************************************************

   moveSuspensionToPressuresConcurrent()
   Controls inflate/deflate of bags to match the target pressures
   The difference with this V2 function is that it opens
   the required solenoids simultaneously instead of consecutively

 *****************************************************************/
void moveSuspensionToPressuresConcurrent()
{
  
  //Inflate bags that need to be inflated first
  int inflateTimeFront = 250; //Time to keep the inflate solenoids open for front bags
  int deflateTimeFront = 100; //Time to keep the deflate solenoids open for front bags
  int inflateTimeRear = 250; //Time to keep the inflate solenoids open for rear bags
  int deflateTimeRear = 100; //Time to keep the deflate solenoids open for rear bags
  
  int margin = 3; //Stop criteria within 3 psi of target plus or minus
  bool targetReached = false;
  bool inflateFront = false;
  bool inflateRear = false;

  //Inflate what needs to go up first
  if (FLPressurePSI < (targetFLPressure - margin))
  {
    //Inflate FL
    digitalWrite(FLINPIN, LOW);
    inflateFront = true;
  }

  if (FRPressurePSI < (targetFRPressure - margin))
  {
    //Inflate FR
    digitalWrite(FRINPIN, LOW);
  }

  if (RRPressurePSI < (targetRRPressure - margin))
  {
    //Inflate RR
    digitalWrite(RRINPIN, LOW);
    inflateRear = true;
  }

  if (RLPressurePSI < (targetRLPressure - margin))
  {
    //Inflate RL
    digitalWrite(RLINPIN, LOW);
    inflateRear = true;
  }

  delay(inflateTimeFront);

  if(inflateFront)
  {
    //Leave the solenoids open for the right time and close both
    //delay(inflateTimeFront);
    digitalWrite(FLINPIN, HIGH);
    digitalWrite(FRINPIN, HIGH);
    inflateFront = false;
  }

  if(inflateRear)
  {
    //Leave the solenoids open the right time and close them
    //delay(inflateTimeRear);
    digitalWrite(RRINPIN, HIGH);
    digitalWrite(RLINPIN, HIGH);
    inflateRear = false;
  }
/*
  //then deflate what is out of range
  if(FLPressurePSI > targetFLPressure + margin)
  {
    //debug("Deflating FL: ");
    //debug(FLPressurePSI);
    //debug(" > ");
    //debug(targetFLPressure);
    //debug(" + ");
    //debugln(margin);
    digitalWrite(FLEXPIN, LOW);
    delay(deflateTimeFront);
    digitalWrite(FLEXPIN, HIGH);
  }

  if(FRPressurePSI > targetFRPressure + margin)
  {
    //debug("Deflating FL: ");
    //debug(FLPressurePSI);
    //debug(" > ");
    //debug(targetFLPressure);
    //debug(" + ");
    //debugln(margin);
    digitalWrite(FREXPIN, LOW);
    delay(deflateTimeFront);
    digitalWrite(FREXPIN, HIGH);
  }

  if(RRPressurePSI > targetRRPressure + margin)
  {
    //debug("Deflating FL: ");
    //debug(FLPressurePSI);
    //debug(" > ");
    //debug(targetFLPressure);
    //debug(" + ");
    //debugln(margin);
    digitalWrite(RREXPIN, LOW);
    delay(deflateTimeFront);
    digitalWrite(RREXPIN, HIGH);
  }

  if(RLPressurePSI > targetRLPressure + margin)
  {
    //debug("Deflating FL: ");
    //debug(FLPressurePSI);
    //debug(" > ");
    //debug(targetFLPressure);
    //debug(" + ");
    //debugln(margin);
    digitalWrite(RLEXPIN, LOW);
    delay(deflateTimeFront);
    digitalWrite(RLEXPIN, HIGH);
  }
*/
  //Delay to leave the solenoids open
  //delay(inflateTimeFront);
  //Close everything
  //closeAllSolenoids();
  //Delay to stabilize pressures before reading
  //delay(100);
  //Update the pressures
  //readPressures();


  //delay(250);

}
