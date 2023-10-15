/*
 * Really basic RS485 Master/Slave proof of concept 
 * before moving on to more complicated stuff
 * 
 * Master sends characters "L" and "H" in 1 sec intervals
 * The arduino at the other end blinks its LED based on value (High or Low)
 * 
 * The slave arduino uses the example Communications-->PhysicalPixel
 */

#define LED   13        //LED pin
#define MASTER_ENABLE 4 //Enable pin for DE and RE (LOW = Rx, HIGH = Tx) 


void setup() {

  pinMode(LED, OUTPUT);   //Make LED pin an output to control it
  digitalWrite(LED, LOW); // LED off

  pinMode(MASTER_ENABLE, OUTPUT);   //Needs to be an output
  digitalWrite(MASTER_ENABLE, HIGH); //Initialize to HIGH to transmis

  Serial.begin(9600);     // Initialize serial connection
  while(!Serial);         // Wait for it to initialize
}

void loop() {
  //MAX485 enable pins should be HIGH to transmit
  digitalWrite(LED, HIGH); //Turn LED on as a visual indicator we are transmissing
  Serial.println('H');    //Send character 'A'
  Serial.flush();  //Wait for transmission to be complete
  delay(1000);
  Serial.println('L');
  digitalWrite(LED, LOW); //LED off, we are done transmitting
  delay(1000);             //Delay so that we can see the LED
 
}
