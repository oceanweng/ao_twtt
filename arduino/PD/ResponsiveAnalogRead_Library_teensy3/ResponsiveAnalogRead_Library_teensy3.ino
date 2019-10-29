unsigned long time;
unsigned long between;
unsigned long last;
unsigned int  AnaVal;

byte  AnaVal_H;
byte  AnaVal_L;

// include the ResponsiveAnalogRead library

#include <TimerOne.h>
#include <ResponsiveAnalogRead.h>

// define the pin you want to use

const int CV_POT_PIN = 16;


// make a ResponsiveAnalogRead object, pass in the pin, and either true or false depending on if you want sleep enabled
// enabling sleep will cause values to take less time to stop changing and potentially stop changing more abruptly,
//   where as disabling sleep will cause values to ease into their correct position smoothly and more accurately

ResponsiveAnalogRead AnalogCVpot(CV_POT_PIN, true, 0.01);

// the next optional argument is snapMultiplier, which is set to 0.01 by default
// you can pass it a value from 0 to 1 that controls the amount of easing
// increase this to lessen the amount of easing (such as 0.1) and make the responsive values more responsive
// but doing so may cause more noise to seep through if sleep is not enabled

void setup() {
  // begin serial so we can see analog read values through the serial monitor
  Serial.begin(9600);
  while (!Serial)
  {
    }
  Timer1.initialize(1000);         
  Timer1.attachInterrupt(AnRead); // to run every initialize(xxx); seconds

  analogReadResolution(12);
  AnalogCVpot.enableSleep();
  AnalogCVpot.enableEdgeSnap();
  AnalogCVpot.setAnalogResolution(4096);
  
}


void AnRead(void) // update the ResponsiveAnalogRead object interrupt.
{
  last = time;
  time = micros();
  between = time - last;
  
 
  AnalogCVpot.update();
  AnaVal=AnalogCVpot.getValue();


  // Bit shift CALC  High side ASCII / LOW side ASCII
  AnaVal_H =  (unsigned int)AnaVal >> 8;
  AnaVal_L = AnaVal;

  Serial.print("$"); 
  //Serial.print(AnaVal,DEC);
  Serial.write(AnaVal_H);

  Serial.write(AnaVal_L);

  Serial.print(",");
  Serial.write(between);
  Serial.println();
}


void loop() {   
 //ALL Analog read are interrupt. Loop has no Code. 
}
