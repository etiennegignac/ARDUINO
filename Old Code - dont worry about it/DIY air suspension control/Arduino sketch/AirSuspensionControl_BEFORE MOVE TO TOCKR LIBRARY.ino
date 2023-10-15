/*---------------------------- CONFIG -------------------------
---------------------------------------------------------------*/
#define MODBUS_SLAVE_ID 10 //Slave ID of this MODBUS module
#define INTERRUPT_PIN_FROM_MPU6050  0 //Pin number for MPU6050 interrupt
#define ALL_BAUD 38400  //Baud rate for both Serial and Modbus server

/* MODBUS REFRESHER
 *  
 *  Coils: 1 bit registers, usually to control discrete outputs on the server, read / write
 *  Discrete inputs: 1 bit registers, used as inputs for the client, read only
 *  Input registers: 16 bit registers used for inputs, read only
 *  Holding registers: 16 bit registers, general use, read / write
 *  
 *  COILS
 *  ...
 *  DISCRETE INPUTS
 *  ...
 *  INPUT REGISTERS
 *  0x00    Truck pitch (1/2 - High word)
 *  0x01    Truck pitch (2/2 - Low word)
 *  0x02    Truck roll (1/2 - High word)
 *  0x03    Truck roll (2/2 - Low word)
 *  
 */

 
/*---------------------------------------------------------------
---------------------------------------------------------------*/

#define BUFFER_LENGTH 32
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include <MPU6050_tockn.h>
#include "Wire.h"
#include "ArduinoRS485.h" // ArduinoModbus depends on the ArduinoRS485 library
#include "ArduinoModbus.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 mpu();
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

bool blinkState = false;

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
float pitch;       // Value extracted from ypr[3] for easy of manipulation
float roll;        // Value extracted from ypr[3] for easy manipulation

// MODBUS related variables
const int numCoils = 15;
const int numDiscreteInputs = 10;
const int numHoldingRegisters = 10;
const int numInputRegisters = 10;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(200000); // 400kHz I2C clock (200kHz if CPU is 8MHz)

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(ALL_BAUD);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //while (Serial.available() && Serial.read()); // empty buffer

    // start the Modbus RTU server
    if (!ModbusRTUServer.begin(10,38400)) {
      Serial.println("Failed to start Modbus RTU Server!");
      while (1);
    }

    //Configure the Modbus coils/registers
    //ModbusRTUServer.configureCoils(0x00, numCoils);
    //ModbusRTUServer.configureDiscreteInputs(0x00, numDiscreteInputs);
    //ModbusRTUServer.configureHoldingRegisters(0x00, numHoldingRegisters);
    ModbusRTUServer.configureInputRegisters(0x00, numInputRegisters);
    

    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    //mpu.setXGyroOffset(220);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    mpu.calcGyroOffsets();
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_FROM_MPU6050), dmpDataReady, RISING); // Attach interrupt to pin
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_BUILTIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
      Serial.println("DMP not ready...");
      return;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
      /*
      Serial.print("Waiting for interrupt: ");
      Serial.print(mpuInterrupt);
      Serial.print(" fifoCount: ");
      Serial.print(fifoCount);
      Serial.print("/");
      Serial.println(packetSize);
      */
      
      //Poll for Modbus requests
      ModbusRTUServer.poll();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset FIFO so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);//*/

            //Extract pitch and roll values from the array (no yaw for truck control)
            pitch = ypr[1] * 180 / M_PI;
            roll = ypr[2] * 180 / M_PI;
            
            /*Serial.print("Pitch:\t");
            Serial.print(pitch);
            Serial.print("\t");
            Serial.print("Roll:\t");
            Serial.println(roll);//*/

            //Populate Modbus coils / registers
            // Input registers

            //ORIGINAL EXAMPLE TO COPY BACK WHEN IT SHITS THE BED FOR SOME REASON
            /*float a = 12.34;
            byte dataArray[4] = {
              ((uint8_t*)&a)[0],
              ((uint8_t*)&a)[1],
              ((uint8_t*)&a)[2],
              ((uint8_t*)&a)[3]
           };*/
/*
           uint16_t dataArray[2] = {
            ((uint16_t*)&pitch)[0],
            ((uint16_t*)&pitch)[1]
           };*/

           //Float values are 32 bits, break down values into 16 bits chunks to send it on Modbus

            
            //Serial.println(((uint16_t*)&pitch)[0], BIN);
            //Serial.println(((uint16_t*)&pitch)[1], BIN);
            //float test = 1234.1;
            //Serial.println(((uint16_t*)&test)[0]); //DEC = 17203
            //ModbusRTUServer.inputRegisterWrite(0x00, ((uint16_t*)&test)[0]); //Pitch (High word)
            //ModbusRTUServer.inputRegisterWrite(0x01, 5678); //Pitch (Low word)

            //Pitch (0x00 and 0x01)
            ModbusRTUServer.inputRegisterWrite(0x00, ((uint16_t*)&pitch)[0]); //Pitch (High word)
            ModbusRTUServer.inputRegisterWrite(0x01, ((uint16_t*)&pitch)[1]); //Pitch (Low word)

            //Roll (0x02 and 0x03)
            ModbusRTUServer.inputRegisterWrite(0x02, ((uint16_t*)&roll)[0]); //roll (High word)
            ModbusRTUServer.inputRegisterWrite(0x03, ((uint16_t*)&roll)[1]); //roll (Low word)
/*
            float test;
            ((int16_t*)&test)[0] = dataArray[0];
            ((int16_t*)&test)[1] = dataArray[1];

            Serial.println(test);*/
        #endif
    }
}
