/*
 * Really basic RS485 Master/Slave proof of concept 
 * before moving on to more complicated stuff
 * 
 * Master sends character "A", 
 * 
 * Slave just has to read it and turn on LED if there is a match
 * 
 * This is all simplex.
 */



#define LED   13        //LED pin
#define MASTER_ENABLE 4 //Enable pin for DE and RE (LOW = Rx, HIGH = Tx) 


void setup() {
  pinMode(LED, OUTPUT);   //Make LED pin an output to control it
  digitalWrite(LED, LOW); // LED off

  pinMode(MASTER_ENABLE, OUTPUT);   //Needs to be an output
  digitalWrite(MASTER_ENABLE, LOW); //Initialize to LOW

  Serial.begin(9600);     // Initialize serial connection
  while(!Serial);         // Wait for it to initialize

}

void loop()
{
   //MAX485 enable pins should be LOW to receive
   while(Serial.available())
   {
      if(Serial.read() == 'A')
      {
        digitalWrite(LED, HIGH); // Turn led on
        delay(1000);
        digitalWrite(LED, LOW); // Turn LED off
        delay(2000);
         
      }
   }


  
}
