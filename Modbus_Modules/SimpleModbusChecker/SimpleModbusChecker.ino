/*---------------------------- CONFIG -------------------------
---------------------------------------------------------------*/
#define MODBUS_SLAVE_ID 10 //Slave ID of the module to probe
#define ALL_BAUD 38400  //Baud rate for both Serial and Modbus server

/*
  Modbus RTU Client Kitchen Sink

  This sketch creates a Modbus RTU Client and demonstrates
  how to use various Modbus Client APIs.

  Circuit:
   - MKR board
   - MKR 485 shield
     - ISO GND connected to GND of the Modbus RTU server
     - Y connected to A/Y of the Modbus RTU server
     - Z connected to B/Z of the Modbus RTU server
     - Jumper positions
       - FULL set to OFF
       - Z \/\/ Y set to ON

  created 18 July 2018
  by Sandeep Mistry
*/

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

int counter = 0;

void setup() {
  Serial.begin(ALL_BAUD);
  //while (!Serial);

  Serial.println("Modbus RTU Client Kitchen Sink");

  // start the Modbus RTU client
  if (!ModbusRTUClient.begin(ALL_BAUD)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }
}

void loop() {
  //writeCoilValues();

  readCoilValues();

  //readDiscreteInputValues();

  //writeHoldingRegisterValues();

  //readHoldingRegisterValues();

  //readInputRegisterValues();

  counter++;

  //delay(5000);
  //Serial.println();
}

void writeCoilValues() {
  // set the coils to 1 when counter is odd
  byte coilValue = ((counter % 2) == 0) ? 0x00 : 0x01;

  Serial.print("Writing Coil values ... ");

  // write 10 Coil values to (slave) id , address 0x00
  ModbusRTUClient.beginTransmission(MODBUS_SLAVE_ID, COILS, 0x00, 10);
  for (int i = 0; i < 10; i++) {
    ModbusRTUClient.write(coilValue);
  }
  if (!ModbusRTUClient.endTransmission()) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");
  }

  // Alternatively, to write a single Coil value use:
  // ModbusRTUClient.coilWrite(...)
}

void readCoilValues() {
  Serial.print("Reading Coil values ... ");

  // read 10 Coil values from (slave) id 42, address 0x00
  /*
  if (!ModbusRTUClient.requestFrom(MODBUS_SLAVE_ID, COILS, 0x00, 10)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");

    while (ModbusRTUClient.available()) {
      Serial.print(ModbusRTUClient.read());
      Serial.print(' ');
    }
    Serial.println();
  }*/
  Serial.println(ModbusRTUClient.coilRead(MODBUS_SLAVE_ID, 0x00));

  // Alternatively, to read a single Coil value use:
  // ModbusRTUClient.coilRead(...)
}

void readDiscreteInputValues() {
  Serial.print("Reading Discrete Input values ... ");

  // read 10 Discrete Input values from (slave) id 42, address 0x00
  if (!ModbusRTUClient.requestFrom(MODBUS_SLAVE_ID, DISCRETE_INPUTS, 0x00, 10)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");

    while (ModbusRTUClient.available()) {
      Serial.print(ModbusRTUClient.read());
      Serial.print(' ');
    }
    Serial.println();
  }

  // Alternatively, to read a single Discrete Input value use:
  // ModbusRTUClient.discreteInputRead(...)
}

void writeHoldingRegisterValues() {
  // set the Holding Register values to counter

  Serial.print("Writing Holding Registers values ... ");

  // write 10 coil values to (slave) id 42, address 0x00
  ModbusRTUClient.beginTransmission(MODBUS_SLAVE_ID, HOLDING_REGISTERS, 0x00, 10);
  for (int i = 0; i < 10; i++) {
    ModbusRTUClient.write(counter);
  }
  if (!ModbusRTUClient.endTransmission()) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");
  }

  // Alternatively, to write a single Holding Register value use:
  // ModbusRTUClient.holdingRegisterWrite(...)
}

void readHoldingRegisterValues() {
  Serial.print("Reading Input Register values ... ");

  /*
  // read 10 Input Register values from (slave) id 42, address 0x00
  if (!ModbusRTUClient.requestFrom(42, HOLDING_REGISTERS, 0x00, 10)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");

    while (ModbusRTUClient.available()) {
      Serial.print(ModbusRTUClient.read());
      Serial.print(' ');
    }
    Serial.println();
  }*/

  
  // Alternatively, to read a single Holding Register value use:
  //ModbusRTUClient.holdingRegisterRead(10,0x00)
  
}

void readInputRegisterValues() {
  //Serial.print("Reading input register values ... ");
/*
  // read 10 discrete input values from (slave) id 42,
  if (!ModbusRTUClient.requestFrom(42, INPUT_REGISTERS, 0x00, 10)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("success");

    while (ModbusRTUClient.available()) {
      Serial.print(ModbusRTUClient.read());
      Serial.print(' ');
    }
    Serial.println();
  }*/
  float pitch, roll; //Pitch and roll read from slave
  //uint16_t byteArray[2];

  ((uint16_t*)&pitch)[0] = ModbusRTUClient.inputRegisterRead(MODBUS_SLAVE_ID,0x00); //Pitch high word
  ((uint16_t*)&pitch)[1] = ModbusRTUClient.inputRegisterRead(MODBUS_SLAVE_ID,0x01); //Pitch low word

  ((uint16_t*)&roll)[0] = ModbusRTUClient.inputRegisterRead(10,0x02); //Roll high word
  ((uint16_t*)&roll)[1] = ModbusRTUClient.inputRegisterRead(10,0x03); //Roll low word

  //byteArray[0] = ModbusRTUClient.inputRegisterRead(10,0x00);
  /*int test = ModbusRTUClient.inputRegisterRead(MODBUS_SLAVE_ID,0x00);
  if(test==-1)
    Serial.println(ModbusRTUClient.lastError());
  else
    Serial.println(test);*/
  
  Serial.print("Pitch:\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print("Roll:\t");
  Serial.println(roll);//*/
  // Alternatively, to read a single Input Register value use:
  // ModbusRTUClient.inputRegisterRead(...)
}
