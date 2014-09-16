#include <cmath>
#include "interface.h"

void Motor::turnOnDirPositive()
{
   if (inverted)
      turnOnDir2();
   else
      turnOnDir1();
}

void Motor::turnOnDirNegative()
{
   if (inverted)
      turnOnDir1();
   else
      turnOnDir2();
}

void Motor::invertPolarity(bool invert)
{
   inverted = invert;
}
