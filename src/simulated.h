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

#ifndef SIMULATED_H
#define SIMULATED_H

#include <chrono>
#include <random>
#include "interface.h"

/* A motor+axis simulator.
 *
 * This emulates a motor spinning an axis and exhibiting real-world
 * characteristics such as initial stall and range limited by end switches.
 * All error conditions are logged to stderr.
*/
class SimulatedMotor : public Motor
{
public:
   SimulatedMotor(degrees relativeInitialAngle = 0);
   void turnOff();
   void setPWM(unsigned short duty);
   degrees currentAngle();

   // In verbose mode, the simulator reports various information (starts,
   // stops, duty cycle changes etc.) to stderr. If set to false, the simulator
   // will only report error conditions, such as the axis hitting an end switch.
   void setVerbose(bool verbose);

private:
   void turnOnDir1();
   void turnOnDir2();

   // The heart of the SimulatedMotor state machine: event() should be called
   // just before any part of the motor state is modified. It updates the motor
   // state according to the current parameters and the time elapsed since the
   // previous event.
   void event();

   // engaged: 0 when still, 1 or -1 when energized (depending on direction)
   int engaged = 0;
   // PWM duty cycle
   int duty = 0;
   // whether to simulate the initial stall
   bool initialStall;
   degrees internalAngle;
   std::chrono::steady_clock::time_point lastEvent;
   bool verbose = false;

   const unsigned short minimum_duty = 15;
   const unsigned short maximum_duty = 30;
   const degrees initialAngle = 250;
   const degrees minimum_angle = 230;
   const degrees maximum_angle = minimum_angle + 360 - 40;
   const float rpm_capability = 10.0 / 6.0;

   // If initial stall simulation is enabled, a PWM cycle of at least
   // stall_overcome_duty will be needed for the motor to start moving.
   // Once the motor overcomes the stall, the duty cycle can be lowered.
   const unsigned short stall_overcome_duty = 0;
};


/* The sensor simulator tries to emulate the characteristics of a real-world
 * noisy signal: a normally distributed random value is added to the real
 * angle value and a completely random spike is inserted every now and then.
*/
class SimulatedSensor : public Sensor
{
public:
   SimulatedSensor(SimulatedMotor* driver);
   RawAngle getRawAngle();

private:
   SimulatedMotor* motor;
   std::mt19937_64 generator;
   std::normal_distribution<degrees> normdist{0.0, 0.1};
   unsigned int numberOfReadouts = 0;

   const unsigned int randomSpikePeriod = 233;
   std::uniform_real_distribution<degrees> spikedist{0.0, 360.0};
};

#endif // SIMULATED_H
