/*
 *    mcontrol, declination axis control for PAART, the radiotelescope of
 *              Astronomical Society Vega - Ljubljana
 *
 *    Copyright (C) 2014 Andrej Lajovic <andrej.lajovic@ad-vega.si>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
   virtual RawAngle getRawAngle();
};

#endif // HARDWARE_H
