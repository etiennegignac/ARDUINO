/*
- Sketch: RPMFakerTeensy
- Author: Etienne Gignac-Bouchard
- Data started: 24 Jan 24
- Last modified: 24 Jan 24
- Description: Generates a PWM square signal (0-5 volts, roughly 15% positive duty cycle) to imitate the RPM signal from the crank sensor (with one notch)
                of the cummins 12 valve.  This sketch is meant to run on a Teensy 4.1.  Equivalent code on Arduino UNO does not have enough CPU power to run
                constantly at higher10frequancy PWMs.

*/


//TODO
  //get arduino through logic level conterter
  //Get both arduino and Teensy from 5V power converter (12v->5v)
  //output of arduino on pin 2 of Teensy
  //Debug timing for input interrupt
  //Debug calculated micro difference
  //Output corrected difference to PWM output period.


  //Force period for development
  //120ms = 500 rpm
  //60 = 1000
  //40 = 1500
  //30 = 2000
  //24 = 2500
  //20 = 3000
  //17 = 3500
  //15 = 4000
  //12 = 5000



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

#if !( defined(CORE_TEENSY) || defined(TEENSYDUINO) )
  #error This code is designed to run on Teensy platform! Please check your Tools->Board setting.
#endif

// These define's must be placed at the beginning before #include "Teensy_Slow_PWM.h"
// _PWM_LOGLEVEL_ from 0 to 4
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_      0

#define USING_MICROS_RESOLUTION       true
#define USING_HW_TIMER_INTERVAL_MS    true

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "Teensy_Slow_PWM.h"

#define LED_OFF             HIGH
#define LED_ON              LOW

#ifndef LED_BUILTIN
  #define LED_BUILTIN       13
#endif

//Leave as-is for PTeeny_Slow_PWM library
#if defined(__IMXRT1062__)
  // For Teensy 4.0 and 4.1
  // Don't change these numbers to make higher Timer freq. System can hang
  #define HW_TIMER_INTERVAL_MS        0.0333f
  #define HW_TIMER_INTERVAL_FREQ      30000L
#elif defined(__MK66FX1M0__)
  // For Teensy 3.6
  // Don't change these numbers to make higher Timer freq. System can hang
  #define HW_TIMER_INTERVAL_MS        0.05f
  #define HW_TIMER_INTERVAL_FREQ      20000L
#else
  // Don't change these numbers to make higher Timer freq. System can hang
  #define HW_TIMER_INTERVAL_MS        0.1f
  #define HW_TIMER_INTERVAL_FREQ      10000L
#endif

// Init Teensy timer TEENSY_TIMER_1
TeensyTimer ITimer(TEENSY_TIMER_1);

// Init Teensy_SLOW_PWM, each can service 16 different ISR-based PWM channels
Teensy_SLOW_PWM ISR_PWM;

//This is the Teeny_Slow_PWM imer interrups handler
//////////////////////////////////////////////////////
void TimerHandler()
{ 
  ISR_PWM.run();
}

volatile uint32_t startMicros = 0;

// You can assign pins here. Be carefull to select good pin to use or crash
uint32_t PWM_Pin    = LED_BUILTIN;
#define INPUT_PIN   2 //Digital pin 2 on te Teensy will handle interrupts from the sensor through a logic level converter to make it 3.3v logic level

// Timer number used to identify associated timer
int timerNum;

volatile uint32_t startInputMicros = 0;
volatile uint32_t currentInputMicros = 0;

////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  //while (!Serial);

  delay(200);

  /////////////////////////////////////////////////////////////////////////////
  // Setup PWM output timer

  // Interval in microsecs
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler))
  {
    startMicros = micros();
    Serial.print(F("Starting ITimer OK, micros() = ")); Serial.println(startMicros);
  }
  else
    Serial.println(F("Can't set ITimer correctly. Select another freq. or interval"));

  //Using period in microsecs resolution, give an initial value so that we can just modify it later
  timerNum = ISR_PWM.setPWM_Period(13, 120000, 50.0); //120ms = 500

  //////////////////////////////////////////////////////////////////////////////
  // Setup input pin and interrupts
  pinMode(INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), inputIntHandler, RISING);  //Looking for a rising signal on pin 2

}

void inputIntHandler()
{
  //noInterrupts();
  //Serial.print("Input interrupt at micros:");
  //Serial.println(micros());
  //delay(1000);
  //interrupts();

  //If this is the first interrupt to calculate:
  if(startInputMicros == 0)
    startInputMicros = micros(); //save current value for next time around

  else //We have a value for start micros, use this value to calculate
  {
    //Debug
    Serial.println((micros() - startInputMicros) / 1000);

    //Set PWM output 
    //Reset for next calculation
    startInputMicros = 0;
  }
}

void loop()
{
  //while(1);
  //Serial.println("Waiting in loop...");
  //delay(1000);  
  //ISR_PWM.modifyPWMChannel_Period(channelNum, PWM_Pin, 40, 15.0); // 120 ms period = 500 RPMs 
  //ISR_PWM.modifyPWMChannel_Period(channelNum, PWM_Pin, 24, 15.0); // 24 ms period = 2500 RPMs (41.6 Hz)

  //delay(2000);
}