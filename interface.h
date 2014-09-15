#ifndef INTERFACE_H
#define INTERFACE_H

class Sensor
{
public:
   virtual float getRawAngle() = 0;
   float getCookedAngle();
   void setZeroPosition(float rawAngleAtZero);
   void invertPolarity(bool invert);

protected:
   float zeroPosition = 0;
   bool inverted = false;
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

float mod360(float value);

#endif // INTERFACE_H
