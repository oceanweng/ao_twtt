
unsigned long time;
unsigned long between;
unsigned long last;
unsigned int  AnaVal;
int incheck = 0;
int intensity[15] = {};
//int intensitycheck[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//int intensitycheck[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int intensitycheck[15] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
byte  AnaVal_H;
byte  AnaVal_L;

int detectsize = sizeof (intensity)/sizeof(intensity[0]);
int intensitymid = 2000;

int printindicate = 0;
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
  Timer1.initialize(100000);         
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
  incheck = 0;
 
  AnalogCVpot.update();
  AnaVal=AnalogCVpot.getValue();

  //Serial.print("current intensity:"); 
  //Serial.print(AnaVal); 
  //Serial.println();

    for ( int i = 0; i < detectsize-1; i++ ) // initialize elements of array n to 0 
   {
      intensity[i] = intensity[i+1];
   }
   
    if (AnaVal > intensitymid) 
    {
      intensity[detectsize-1] = 1; 
    }
    else 
    {
      intensity[detectsize-1] = 0;
    }


    if (printindicate == 1)  
    {
      for ( int i = 0; i < detectsize; i++ )
        {
          if (i % 10 == 0 && i != 0)
            { 
              Serial.print(",");
            }
          Serial.print(intensity[i]); 
        }
      Serial.println();  
    }
    
    for ( int i = 0; i < detectsize; i++ ) // initialize elements of array n to 0 
   {

      
      if (intensity[i] == intensitycheck[i])
      {
        incheck = incheck + 1;
        //Serial.println(incheck); 
        //Serial.println(sizeof (intensity));
        }
      if (incheck == (sizeof (intensity)/sizeof(intensity[0])))
      {
        Serial.println("Got"); 
        }
   }

  // Bit shift CALC  High side ASCII / LOW side ASCII
  //AnaVal_H =  (unsigned int)AnaVal >> 8;
  //AnaVal_L = AnaVal;

  //Serial.print("$"); 
  //Serial.print(AnaVal,DEC);
  //Serial.write(AnaVal_H);

  //Serial.write(AnaVal_L);

  //Serial.print(",");
  //Serial.write(between);
  //Serial.println();
  //Serial.println(between);
}


void loop() {   
 //ALL Analog read are interrupt. Loop has no Code. 
}
