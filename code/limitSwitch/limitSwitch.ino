#include <Arduino.h>

#include "LimitSwitch.h"

LimitSwitch mySwitch(20);

void setup()
{
  
}

void loop()
{
  if(mySwitch.isPressed())
  {
    //zastav motor nebo se to posere!
  }
  else
  {
    //dobry je to!
  }
}
