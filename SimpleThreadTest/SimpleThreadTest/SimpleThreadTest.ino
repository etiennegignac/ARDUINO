#include <ThreadController.h>
#include <StaticThreadController.h>
#include <Thread.h>

Thread theThread = Thread();
int i = 0;

void myFunction()
{
  Serial.println("Thread running");
  while(1)
  {
    Serial.println("Callback!");
    delay(5000);
    //Serial.println("Leaving callback!");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  theThread.onRun(myFunction);
  theThread.setInterval(1000);
}

void loop() {
  
  if (theThread.shouldRun())
  {
    theThread.run();
  }
  
  // put your main code here, to run repeatedly:
  
  Serial.print("The loop says: ");
  Serial.println(i);

  delay(2000);
}
