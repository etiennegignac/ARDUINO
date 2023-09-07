#define LOG_DEBUG 1  //Change this for 1 to enable serial monitor debugging
#if LOG_DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define NOT_MEASURED 9999

/*
- Sketch: airSuspensionHelper
- Author: Etienne Gignac-Bouchard
- Data started: 7 Sep 23
- Last modified: 7 Sep 23
- Description: Program used to convert the RPM sensor on the 12 valve cummins (one notch) to interface with the Holley dash (8 cylinder)

*/





/***********************************************

        GLOBAL VARIABLES

************************************************/

volatile unsigned long lastTime = 0, thisTime = 0, timeDiff = 0;  // Variables used to calclate the time difference in one interval (between 2 interrupts)
volatile int nbIntervals = 0;                                     //Number of intervals counted

/***********************************************
Need:

1 x digital input connected to 12 valve engine speed sensor to count actual RPMs
1 x digital PWM output to feed the Holley dash


        HARDWARE PIN MAPPING


              INPUTS
      (PIN)         (Description)
      A0            Analog input from pedal TPS (foot off = 0V).  Has to be able to to analog to digital.
                    


              OUTPUTS
      (PIN)         (Description)
      3            PWM Output to servo

************************************************/
#define SENSOR_IN 2  //This pin needs to be able to have interrupts
#define DASH_OUT 1

/***********************************************

        SETUP

************************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Configure SENSOR pin as input, add PULLUP or PULLDOWN as required, TODO resistor
  pinMode(SENSOR_IN, INPUT);

  //Attach interrupt, TODO: double check that falling is the right trigger
  attachInterrupt(digitalPinToInterrupt(SENSOR_IN), foundNotch, FALLING);

  //Configure DASH pin as output
  pinMode(DASH_OUT, OUTPUT);
}

/***********************************************

         foundNotch

- Called when sensor pin is interrupted.  Just need to store the 

************************************************/
void foundNotch() {
  noInterrupts();  //Disable interrupts during this time.  What happens if this takes too long and influences the micros() function??

  if (lastTime == 0 && thisTime == 0)  //This is the first interrupt
  {
    //Store time as lastTime used for calculation in the second interrupts
    lastTime = micros();
  } else if (lastTime != 0 && thisTime == 0)  //This is the second interrupt
  {
    //Store time as this time to calculate interval with lastTime
    thisTime = millis();
    timeDiff = thisTime - lastTime;  //Actual time diff in micro-seconds

    nbIntervals++;  //Increase the number of intervals we have measured
  } else            //This is just another interval
  {
    lastTime = thisTime;             //Used last measurement as lastTime
    thisTime = millis();             //current time
    timeDiff = thisTime - lastTime;  //Actual time diff in micro-seconds
    nbIntervals++;                   //Increase the number of intervals measured
  }

  interrupts();  //Re-enables interrupts
}

/***********************************************

         LOOP

- Assume RPMs between 0 and 4100
- One notch: minimum time between notches = 4100 Rot/min * (1min/60sec) = 68.3333 Rot/sec = 0.01463 sec/rot
- Max time between rot at 500 rpms: 500 Rot/min *(1min/60sec) = 8.3333 Rot/sec = 0.12 sec/rot

************************************************/
void loop() {
  if (nbIntervals % 5 == 0)  //At each multiple of 5 intervals
  {
    debug("Got timeDiff : ");
    debug(timeDiff);
    debug(" -- RPMs: ");
    debugln(timeDiff * 60 / 1000);
    //Write PWM value here
  }
}