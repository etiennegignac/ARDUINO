#define OUTPUT_PIN 0  //Pin to connect speaker/relay on


void setup() {
  // put your setup code here, to run once:

  //Make LED pin output to show what we selected during debug
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(OUTPUT_PIN, OUTPUT);  //Pin 1 is the output to our speaker / relay

  Serial.begin(9600);

  Serial.println("440 Sqn prank module - IMPROVED VERSION!!");
}


void flashLED(int nbTimes) {
  int i = 0;

  while (i < nbTimes) {

    //Assume start with LED off
    digitalWrite(LED_BUILTIN, LOW);   //Turn LED on
    delay(1000);                      //one sec delay
    digitalWrite(LED_BUILTIN, HIGH);  //Turn LED off
    delay(1000);
    i++;
  }
}

void playCucaracha() {
  unsigned long prevMillis = millis();

  //test, PWM for 180 ms

  while (millis() - prevMillis < 1000) {

    // PWM test at 500Hz, total period = 1 / 500 = 0.002. Therefore 50 % duty cycle signal switch at 0.002 / 2 = 0.001 = 1ms digitalWrite(OUTPUT_PIN, LOW);
    delay(1);
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(1);
  }

  prevMillis = millis();

  while (millis() - prevMillis < 1000) {

    // PWM test at 500Hz, total period = 1 / 500 = 0.002. Therefore 50 % duty cycle signal switch at 0.002 / 2 = 0.001 = 1ms digitalWrite(OUTPUT_PIN, LOW);
    delay(1);
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(1);
  }

  prevMillis = millis();

  while (millis() - prevMillis < 1000) {

    // PWM test at 500Hz, total period = 1 / 500 = 0.002. Therefore 50 % duty cycle signal switch at 0.002 / 2 = 0.001 = 1ms digitalWrite(OUTPUT_PIN, LOW);
    delay(1);
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(1);
  }
  /*
  digitalWrite(OUTPUT_PIN, LOW);
  delay(100);
  digitalWrite(OUTPUT_PIN, HIGH;)
    delay(180);
  digitalWrite(OUTPUT_PIN, LOW);
  delay(100);
  digitalWrite(OUTPUT_PIN, HIGH;)
    delay(180);
  digitalWrite(OUTPUT_PIN, LOW);
  delay(100);
  digitalWrite(OUTPUT_PIN, HIGH;)
    delay(180);
  */
}




void loop() {

  int prevMillis = millis();

  while (millis() - prevMillis < 30) {
    digitalWrite(OUTPUT_PIN, LOW);
    delay(1);
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(1);
  }

  delay(180);

  prevMillis = millis();

  while (millis() - prevMillis < 30) {
    digitalWrite(OUTPUT_PIN, LOW);
    delay(1);
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(1);
  }

  delay(180);

  prevMillis = millis();

  while (millis() - prevMillis < 30) {
    digitalWrite(OUTPUT_PIN, LOW);
    delay(1);
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(1);
  }

  delay(180);

  prevMillis = millis();

  while (millis() - prevMillis < 400) {
    digitalWrite(OUTPUT_PIN, LOW);
    delay(1);
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(1);
  }

  delay(230);

  prevMillis = millis();

  while (millis() - prevMillis < 30) {
    digitalWrite(OUTPUT_PIN, LOW);
    delay(1);
    digitalWrite(OUTPUT_PIN, HIGH);
    delay(1);
  }

  delay(450);
  //playCucaracha();
}
