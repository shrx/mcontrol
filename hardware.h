#ifndef HARDWARE_H
#define HARDWARE_H

#include "interface.h"

class HardwareMotor : public Motor
{
public:
   HardwareMotor(int setPin1, int setPin2, int setPinPWM);
   virtual void turnOff();
   virtual void setPWM(unsigned short duty);

protected:
   virtual void turnOnDir1();
   virtual void turnOnDir2();

   int pin1;
   int pin2;
   int pinPWM;
};


class HardwareSensor : public Sensor
{
public:
   virtual float getRawAngle();
};

#endif // HARDWARE_H
