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

/* Support for real hardware: an AS5048A Magnetic Rotary Encoder chip
 * connected to the SPI bus and a motor connected to a two-relay one-transistor
 * H-bridge. The two relays control the direction of the motor and the
 * transistor serves to control the power via PWM.
 *
*/
class HardwareMotor : public Motor
{
public:
   /* Create a HardwareMotor instance.
    *
    * pin1: GPIO pin controlling relay1
    * pin2: GPIO pin controlling relay2
    * pinPWM: GPIO pin for pinPWM
    *
    * All pin nubers are according to the wiringPi library.
   */
   HardwareMotor(int setPin1, int setPin2, int setPinPWM);

   // Turn off both relays, cutting the power to the motor.
   virtual void turnOff();

   // Set PWM duty cycle in percent.
   virtual void setPWM(unsigned short duty);

protected:
   // Spin the motor in direction 1 (hardware dependent).
   virtual void turnOnDir1();

   // Spin the motor in direction 2 (hardware dependent).
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
