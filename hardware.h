#ifndef HARDWARE_H
#define HARDWARE_H

#include "interface.h"

class HardwareMotor : public Motor
{
public:
   HardwareMotor(int setPin1, int setPin2, int setPinPWM);
   virtual void turnOff() const;
   virtual void setPWM(unsigned short duty) const;

protected:
   virtual void turnOnDir1() const;
   virtual void turnOnDir2() const;

   int pin1;
   int pin2;
   int pinPWM;
};


class HardwareSensor : public Sensor
{
public:
   virtual RawAngle getRawAngle() const;
};

#endif // HARDWARE_H
