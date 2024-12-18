/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = A0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  //Serial.begin(9600);
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  //Serial.print("APP value: ");
  //Serial.print(val); //original: 0-1024....checked on pedal = 153 - 819 (with APP1), also = 73 - 405 (on APP2 with tighter range)
  val = map(val, 73, 405, 9, 60);     // scale it for use with the servo (found min value to pick up the slack is 9 and max at pump full travel is 60, 55 hsa a bit of play still.  Dont want to push it too hard)
  //Serial.print("Servo mapped value: ");
  //Serial.print(":");
  //Serial.println(val);
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}
