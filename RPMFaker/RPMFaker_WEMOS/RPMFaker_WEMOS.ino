/*
  This sketch takes input from the dodge engine speed sensor (crankshaft position sensor) and outputs a PWM signal compatible with the Ford.

  THIS IS FOR A WEMOS D1 MINI
  
  Arduino UNO will only do Fast PWM on pin D3 or D11.  D11 is toggle only (50% duty cycle) so we have to use D3

  Arduino UNO can onl have interrupts on digital pins D2 and D3.  Because we are using D3 for output, we need to use D2 as input from crank sensor

  FastPWMPin library is here: https://github.com/maxint-rd/FastPwmPin

  RPM from 500 to 5000 RPMs mean:

  500 rotations per minute = 500/60 = 8.3333333333 rotations per second = 8.33333333 Hz x 4 = 33.3333333 Hz for Holley
  5000 rotations per minute = 5000/60 = 83.3333333 rotations per second = 83.3333333 Hz x 4 = 333.3333333 Hz for Holley

  Wemos minimum 100 Hz / 4 = 25 rotations per second = 25 x 60 = 1000 rpm
  
*/



unsigned long currentTime = 0;  //Var used to store time between interrupts
unsigned long lastTime = 0;

int sensorPin = 2;            //Pin used to "read" cummins engine speed sensor
int outputPin = 3;            //Pin used to output new value to truck (or maybe digital dash)

int outputDutyCycle = 10;     //Positive duty cycle of the output PWM signal

long calcFreq = 0.0;          //Frequency of the output to generate

int calcRPM = 0;              //Calculated RPM for debug mostly

// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(9600);

  //Setup sensor pin as input with pullup
  pinMode(sensorPin, INPUT_PULLUP);
  
  
  //Setup interrupts
  attachInterrupt(digitalPinToInterrupt(sensorPin), processInt, RISING); //Sensor is grounded most of the time and goes to Vcc when above one of the two notches on the balancer.
}


void loop()
{
  /*Serial.print("lastTime: ");
  Serial.println(lastTime);
  Serial.print("currentTime: ");
  Serial.println(currentTime);
  Serial.print("Freq: ");
  Serial.println(calcFreq);
  Serial.print("RPMs: ");
  Serial.println(calcRPM);
  Serial.println("");
  */
  
 
}

void processInt()
{
  noInterrupts();
  //We have detected an edge on the sensor

  //lastTime = currentTime;
  currentTime = millis();
 
  Serial.print("New currentTime: ");
  Serial.println(currentTime);

  //calcRPM = 30000/(currentTime - lastTime);
  interrupts();
}

/*
 * Measurements raken
 * 
 * At idle: pico = 25.5 Hz, actual measured laser = 
 * 
 * Conversion formula:
 * inverse frequency for whole pulley = time between 2 notches (1 ÷ 25.5 Hz = 39.2 ms)
 * x 2 = time for one revolution (s / rev) (2 * 39.2 = 78.4 ms)
 * inverse = rev / s. (1 / 78.4 ms = 12.75 rev/s)    X 60 = rev / min (12.75 * 60 = 765 rpm)

 * 
 */


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
