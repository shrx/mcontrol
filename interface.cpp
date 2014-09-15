#include <cmath>
#include "interface.h"

float Sensor::getCookedAngle()
{
   return mod360((inverted ? -1.0 : 1.0) * (getRawAngle() - zeroPosition));
}

void Sensor::setZeroPosition(float rawAngleAtZero)
{
   zeroPosition = rawAngleAtZero;
}

void Sensor::invertPolarity(bool invert)
{
   inverted = invert;
}


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


float mod360(float value)
{
   return value - 360.0 * std::floor(value / 360.0);
}
