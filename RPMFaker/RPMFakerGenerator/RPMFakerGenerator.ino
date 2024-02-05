/*
- Sketch: RPMFakerGenerator
- Author: Etienne Gignac-Bouchard
- Data started: 22 Jan 24
- Last modified: 22 Jan 24
- Description: Generates a PWM square signal (0-5 volts, roughly 15% positive duty cycle) to imitate the RPM signal from the crank sensor (with one notch)
                of the cummins 12 valve

*/



/*****************************************   CALCULATIONS ***********************************************
Stock 12 valve balancer has 1 notch where the metal pulls away from the sensor.  I think this means output signal is always low and goes high when notch flies by the sensor.

Assume we want to measure from 500 to 5000 RPMs.

Lower end:
500 rotations per minute = (500/60) rotations per second = (500/60) second per rotation = (50/6) Hz.  Period is inverse: (6/50) = 0.12 secs 
0.12 seconds = 120 ms --> this is the signal period for the lowest RPM (longuest period)

Higher end:
5000 rotations per minute = (5000/60) rotations per second = (5000/60) second per rotation = (500/6) Hz.  Period is inverse: (6/500) = 0.012 secs
0.012 seconds = 12 ms --> this is the signal period for the highest RPM (shortest period)

SUMMARY
500 RPMs: Period = 120 ms
5000 RPMs: Period = 12 ms
***********************************************************************************************************/

// These define's must be placed at the beginning before #include "AVR_Slow_PWM.h"
// _PWM_LOGLEVEL_ from 0 to 4
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_ 0

//////////////////////////////////////////////////////

#define USING_PWM_FREQUENCY false
#define USING_MICROS_RESOLUTION false
#define USING_MILLIS_RESOLUTION true
#define USE_TIMER_1 true  // required for the PWM library

// Default is true, uncomment to false
//#define CHANGING_PWM_END_OF_CYCLE     false

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "AVR_Slow_PWM.h"
#include <AVR_Slow_PWM.hpp>

// Don't change these numbers to make higher Timer freq. System can hang
#define HW_TIMER_INTERVAL_MS 0.1f
#define HW_TIMER_INTERVAL_FREQ 10000L

volatile uint32_t startMicros = 0;
volatile uint32_t inputStartMillis = 0;
volatile uint32_t inputCurrentMillis = 0;

// Init AVR_Slow_PWM, each can service 16 different ISR-based PWM channels
AVR_Slow_PWM ISR_PWM;

//////////////////////////////////////////////////////

void TimerHandler() {
  ISR_PWM.run();
}



//////////////////////////////////////////////////////

// You can assign pins here. Be carefull to select good pin to use or crash
//uint32_t PWM_Pin    = LED_BUILTIN;
uint32_t PWM_Pin = 5; //Use pin 5 on Arduino for PWM output

uint32_t currentPeriod = 120;  //used to track the current period of the PWM output.  Start at 120ms which is 500 RPMs.
bool goingUp = true;           //true if increasing the RPMs. false if decreasing.  Start by increasing from 0.

// Channel number used to identify associated channel
int timerNum = 0;

////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  while (!Serial);

  delay(500);

  //LED debug
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  //Start with LED off

  //Serial.print(F("\nStarting ISR_Modify_PWM on ")); Serial.println(BOARD_NAME);
  //Serial.println(AVR_SLOW_PWM_VERSION);
  //Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

  // Timer0 is used for micros(), millis(), delay(), etc and can't be used
  // Select Timer 1-2 for UNO, 1-5 for MEGA, 1,3,4 for 16u4/32u4
  // Timer 2 is 8-bit timer, only for higher frequency
  // Timer 4 of 16u4 and 32u4 is 8/10-bit timer, only for higher frequency

  ITimer1.init();

  // Using ATmega328 used in UNO => 16MHz CPU clock ,

  if (ITimer1.attachInterrupt(HW_TIMER_INTERVAL_FREQ, TimerHandler)) {
    Serial.print(F("Attaching interrupt to ITimer1 OK, micros() = "));
    Serial.println(micros());
  } else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));



    // Just to demonstrate, don't use too many ISR Timers if not absolutely necessary
    // You can use up to 16 timer for each ISR_PWM
    //void setPWM(uint32_t pin, uint32_t frequency, uint32_t dutycycle
    // , timer_callback_p StartCallback = nullptr, timer_callback_p StopCallback = nullptr)
    //Serial.print(F("Using PWM Freq = ")); Serial.print(PWM_Freq1); Serial.print(F(", PWM DutyCycle = ")); Serial.println(PWM_DutyCycle1);

#if USING_PWM_FREQUENCY

  // You can use this with PWM_Freq in Hz
  Serial.println("Initial setting up of timer and PWM using freqs");
  ISR_PWM.setPWM(PWM_Pin, 0.0f, 15.0);  //Start at 0 RPMs

#else
#if USING_MICROS_RESOLUTION
  // Or using period in microsecs resolution
  Serial.println("Initial setting up of timer and PWM using micros period");
  //channelNum = ISR_PWM.setPWM_Period(PWM_Pin, PWM_Period1, PWM_DutyCycle1);
#else
  // Or using period in millisecs resolution
  Serial.println("Initial setting up of timer and PWM using millis period");
  //channelNum = ISR_PWM.setPWM_Period(PWM_Pin, 120, 15.0);  //period is uint32
#endif
#endif
}

////////////////////////////////////////////////

void changePWM() {
  /*static uint8_t count = 1;

  float PWM_Freq;
  float PWM_DutyCycle;

  if (count++ % 2)
  {
    PWM_Freq        = PWM_Freq2;
    PWM_DutyCycle   = PWM_DutyCycle2;
  }
  else
  {
    PWM_Freq        = PWM_Freq1;
    PWM_DutyCycle   = PWM_DutyCycle1;
  }

  // You can use this with PWM_Freq in Hz
  if (!ISR_PWM.modifyPWMChannel(channelNum, PWM_Pin, PWM_Freq, PWM_DutyCycle))
  {
    Serial.print(F("modifyPWMChannel error for PWM_Period"));
  }*/
}

void loop() {
  /*If we want to sweep the whole interval in one second: 120 ms - 17 ms = 103 ms
    If we want to update the PWM 10 times per second (let's see if the arduino can keep up)
    103 ms / 10 = 10.3 ms change in period each increase/decrease
  */
ISR_PWM.modifyPWMChannel_Period(timerNum, PWM_Pin, 120, 50.0);
while(1);
while(1)
{
  for(int i = 1; i < 11; i++)
  {
    int tryPeriod = round(120/i);
    ISR_PWM.modifyPWMChannel_Period(timerNum, PWM_Pin, tryPeriod, 50.0);
    delay(500);
  }

  for(int i = 10; i > 0; i--)
  {
    int tryPeriod = round(120/i);
    ISR_PWM.modifyPWMChannel_Period(timerNum, PWM_Pin, tryPeriod, 50.0);
    delay(500);
  }
  delay(500);
}

#if 0
  //We want to trigger a PWM change
  if (goingUp)  //RPMs are going up, meaning that period is going down
  {
    currentPeriod = round(currentPeriod - 21.6);  //substract variation and round to go from float to int

    if (currentPeriod < 12) {
      currentPeriod = 12;  //Force to lowest period
      goingUp = false;     //Switch to going down
    }
  }

  else  // We are RPM going down (therefore PWM period increasing)
  {
    currentPeriod = round(currentPeriod + 21.6);

    if (currentPeriod > 120) {
      currentPeriod = 120;  //Force to 120ms
      goingUp = true;       //Switch back to going up
    }
  }

  //Modify PWM to reflect new period
  if (!ISR_PWM.modifyPWMChannel_Period(timerNum, PWM_Pin, currentPeriod, 15.0))
  {
    Serial.print(F("modifyPWMChannel_Period error!!"));
  }

  //Print new period for debug
  //Serial.print("New period: ");
  //Serial.print(currentPeriod);
  //Serial.println(" ms");
#else
  //Force period for development
  //120ms = 500 rpm
  //60 = 1000
  //40 = 1500
  //30 = 2000
  //24 = 2500
  //20 = 3000
  //17 = 3500
  //15 = 4000
  // 13.33333 = 4500
  //12 = 5000
  ISR_PWM.modifyPWMChannel_Period(timerNum, PWM_Pin, 40, 15.0); // 120 ms period = 500 RPMs 
  //ISR_PWM.modifyPWMChannel_Period(channelNum, PWM_Pin, 24, 15.0); // 24 ms period = 2500 RPMs (41.6 Hz)
#endif

  delay(2000);
}