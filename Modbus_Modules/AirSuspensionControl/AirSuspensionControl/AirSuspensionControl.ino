/*---------------------------- CONFIG -------------------------
  ---------------------------------------------------------------*/
#define MODBUS_SLAVE_ID 1 //Slave ID of this MODBUS module
#define INTERRUPT_PIN_FROM_MPU6050  0 //Pin number for MPU6050 interrupt
#define ALL_BAUD 19200  //Baud rate for both Serial and Modbus server

/* MODBUS REFRESHER

    Coils: 1 bit registers, usually to control discrete outputs on the server, read / write
    Discrete inputs: 1 bit registers, used as inputs for the client, read only
    Input registers: 16 bit registers used for inputs, read only
    Holding registers: 16 bit registers, general use, read / write

    COILS (MASTER CAN READ / WRITE)
    0x00    1 = Suspension is being adjusted, 0 = Suspension is still
    DISCRETE INPUTS (MASTER SHOULD READ ONLY)
    0x00    
    ...
    INPUT REGISTERS (MASTER SHOULD READ ONLY)
    0x00    Truck pitch (1/2 - High word)
    0x01    Truck pitch (2/2 - Low word)
    0x02    Truck roll (1/2 - High word)
    0x03    Truck roll (2/2 - Low word)

*/


/*---------------------------------------------------------------
  ---------------------------------------------------------------*/

#define BUFFER_LENGTH 32
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU6050_tockn.h>
#include "Wire.h"
#include "ArduinoRS485.h" // ArduinoModbus depends on the ArduinoRS485 library
#include "ArduinoModbus.h"


MPU6050 mpu(Wire); //Instance to control the MPU6050

// MODBUS related variables
const int numCoils = 15;
const int numDiscreteInputs = 10;
const int numHoldingRegisters = 10;
const int numInputRegisters = 10;

float pitch, roll;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock (200kHz if CPU is 8MHz)

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(ALL_BAUD);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.begin();

  Serial.println("Calculating gyro offsets, DONT MOVE!");
  //mpu.calcGyroOffsets();
  mpu.calcGyroOffsets(true); //Use 'true' do output in console

  // start the Modbus RTU server
  if (!ModbusRTUServer.begin(MODBUS_SLAVE_ID, ALL_BAUD)) {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1);
  }

  //Configure the Modbus coils/registers
  ModbusRTUServer.configureCoils(0x00, numCoils);
  ModbusRTUServer.configureDiscreteInputs(0x00, numDiscreteInputs);
  ModbusRTUServer.configureHoldingRegisters(0x00, numHoldingRegisters);
  ModbusRTUServer.configureInputRegisters(0x00, numInputRegisters);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{

  //Poll for Modbus requests
  ModbusRTUServer.poll();

  //Get latest from MPU6050
  mpu.update();

  pitch = mpu.getAngleX();
  roll = mpu.getAngleY();

  Serial.print("Pitch:\t");
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
  }
  uint16_t dataArray[2] = {
    ((uint16_t*)&pitch)[0],
    ((uint16_t*)&pitch)[1]
  };//*/

  //Update Coil values
  ModbusRTUServer.coilWrite(0,1); //Temp value, write 1
  ModbusRTUServer.discreteInputWrite(0,1); //Temp value, write 1
  ModbusRTUServer.holdingRegisterWrite(0,2812); //temp
  ModbusRTUServer.inputRegisterWrite(0,2812); //temp
  

  //Float values are 32 bits, break down values into 16 bits chunks to send it on Modbus

  //Pitch (0x00 and 0x01)
  ModbusRTUServer.inputRegisterWrite(0x00, ((uint16_t*)&pitch)[0]); //Pitch (High word)
  ModbusRTUServer.inputRegisterWrite(0x01, ((uint16_t*)&pitch)[1]); //Pitch (Low word)

  //Roll (0x02 and 0x03)
  ModbusRTUServer.inputRegisterWrite(0x02, ((uint16_t*)&roll)[0]); //roll (High word)
  ModbusRTUServer.inputRegisterWrite(0x03, ((uint16_t*)&roll)[1]); //roll (Low word)

  delay(100);
}
