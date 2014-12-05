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

#ifndef INTERFACE_H
#define INTERFACE_H

#include "angles.h"

// Abstract interface for a rotary encoder.
class Sensor
{
public:
   virtual RawAngle getRawAngle() = 0;
};


// Abstract interface for a motor.
class Motor
{
public:
   // Set the motor direction to either positive or negative: the actual
   // meaning of these depends on the implementation.
   void turnOnDirPositive();
   void turnOnDirNegative();

   // Disconnect the motor from the power source.
   virtual void turnOff() = 0;

   // Set the PWM duty cycle in percent.
   virtual void setPWM(unsigned short duty) = 0;

   // Invert the polarity (sense of spinning) of the motor.
   void invertPolarity(bool invert);

protected:
   virtual void turnOnDir1() = 0;
   virtual void turnOnDir2() = 0;
   bool inverted = false;
};

#endif // INTERFACE_H
