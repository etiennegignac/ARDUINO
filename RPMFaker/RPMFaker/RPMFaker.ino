/*
  This sketch takes input from the dodge engine speed sensor (crankshaft position sensor) and outputs a PWM signal compatible wit hthe Ford.

  Arduino UNO will only do Fast PWM on pin D3 or D11.  D11 is toggle only (50% duty cycle) so we have to use D3

  Arduino UNO can onl have interrupts on digital pins D2 and D3.  Because we are using D3 for output, we need to use D2 as input from crank sensor

  FastPWMPin library is here: https://github.com/maxint-rd/FastPwmPin
*/

#include <FastPwmPin.h>

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
  
  //When powering on, sweep the RPM needle from 0 to 5000 and back to 0.
  FastPwmPin::enablePwmPin(outputPin, 5000L, outputDutyCycle);
  delay(1000);
  FastPwmPin::enablePwmPin(outputPin, 0, outputDutyCycle);
  delay(1000);

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
  
  if(calcRPM > 600) //Catch weird exceptions when calculating, has to be over 600 to be valid
  {
    FastPwmPin::enablePwmPin(outputPin, calcRPM, outputDutyCycle);
  }
  else //Put RPMs at zero
  {
    FastPwmPin::enablePwmPin(outputPin, 0, outputDutyCycle);
  }
  
  delay(100);
}

void processInt()
{
  //We have detected an edge on the sensor

  lastTime = currentTime;
  currentTime = millis();
 
  calcRPM = 30000/(currentTime - lastTime);
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
