#include <ArduinoRS485.h>

int counter = 0;

void setup() {
  // put your setup code here, to run once:
  RS485.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  RS485.beginTransmission();
  RS485.print("Hello ");
  RS485.println(counter);
  RS485.endTransmission();

  counter++;

  delay(1000);
}
