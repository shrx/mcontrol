#ifndef INTERFACE_H
#define INTERFACE_H

#include "angles.h"

class Sensor
{
public:
   virtual RawAngle getRawAngle() = 0;
};


class Motor
{
public:
   void turnOnDirPositive();
   void turnOnDirNegative();
   virtual void turnOff() = 0;
   virtual void setPWM(unsigned short duty) = 0;
   void invertPolarity(bool invert);

protected:
   virtual void turnOnDir1() = 0;
   virtual void turnOnDir2() = 0;
   bool inverted = false;
};

#endif // INTERFACE_H
