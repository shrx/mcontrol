#include <wiringPi.h>
#include "hardware.h"

HardwareMotor::HardwareMotor(int setPin1, int setPin2, int setPinPWM) :
   pin1(setPin1), pin2(setPin2), pinPWM(setPinPWM)
{
   pinMode(pin1, OUTPUT);
   pinMode(pin2, OUTPUT);
   // TODO: set PWM status, mode, clock etc.
}

void HardwareMotor::turnOnDir1()
{
   digitalWrite(pin2, LOW);
   digitalWrite(pin1, HIGH);
}

void HardwareMotor::turnOnDir2()
{
   digitalWrite(pin1, LOW);
   digitalWrite(pin2, HIGH);
}

void HardwareMotor::turnOff()
{
   digitalWrite(pin1, LOW);
   digitalWrite(pin2, LOW);
}

void HardwareMotor::setPWM(unsigned short duty)
{
   // TODO: set PWM duty cycle
}

RawAngle HardwareSensor::getRawAngle()
{
   // TODO: read value via SPI
}
