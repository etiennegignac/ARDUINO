/******************************************************************************

Name:         TCaseEncoderPositionSniffer

Description:  Run this sketch and manually move the transfer case motor to
              output all positions to the serial output

Platform:     MKR1010 WIFI

*******************************************************************************/

//Pinout (FOR ARDUINO MKR1010 WIFI)
#define ENCODER_1 5  //Encoder bit 1 / connector pin 5 (purple/yellow)
#define ENCODER_2 4  //Encoder bit 2 / connector pin 3 (brown/white)
#define ENCODER_3 3  //Encoder bit 3 / connector pin 6 (white)
#define ENCODER_4 2  //Encoder bit 4 / connector pin 1 (orange/white)

//Global variables
int lastEncoderValue = 9999; //initial value

/******************************************************************************

Function:     setup()

Arguments:    Nil

Description:  Called once initially to initialize all variables.

Return value: Nil

*******************************************************************************/
void setup()
{
  
  //Initialize serial
  Serial.begin(115200);

  //Wait for Serial output to be available
  while(!Serial);

  Serial.println("Transfer case encoder position sniffer!");
  Serial.println("Make sure the transfer case is in 2WD and motor is not activated before running this");

  //All encoder pins as input pulled up
  pinMode(ENCODER_1, INPUT_PULLUP);
  pinMode(ENCODER_2, INPUT_PULLUP);
  pinMode(ENCODER_3, INPUT_PULLUP);
  pinMode(ENCODER_4, INPUT_PULLUP);
}

void loop()
{
  /* The plan: every loop, read encoder position, if it is different than the one stored in the currentState variable, output it to Serial.
               Start from relaxed 2WD position.  Force motor all the way to the end of the 2WD range, relax it, then move it all the way to
               the end of the 4LO range.  Serial should have all possible positions.  This does not rely on interrupts (no problem with contact bouncing)
  */  

  //Get encoder value. Any leading "0" will be ignored in the final value
  int rawEncoderValue = (digitalRead(ENCODER_1) * 1000) + (digitalRead(ENCODER_2) * 100) + (digitalRead(ENCODER_3) * 10) + digitalRead(ENCODER_4);

  // Compare to last known value see if we came across a new encoder value
  if(rawEncoderValue != lastEncoderValue)
  {
    //Output the new value to Serial
    Serial.print("New encoder: ");
    Serial.println(rawEncoderValue);

    //Save new encoder value as last encoder value
    lastEncoderValue = rawEncoderValue;
  }
}
