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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <chrono>
#include <exception>
#include "angles.h"
#include "interface.h"

class ConfigFileException : public std::exception
{
public:
   ConfigFileException(const std::string& what) : message(what) {}
   inline const char* what() { return message.c_str(); }
   const std::string message;
};

/* Parameters that affect the operation of the controller. Runtime values of
 * these parameters will be read from the configuration file (all of them are
 * required to be explicitly set there).
 */
struct ControllerParams
{
   ControllerParams() = default;
   ControllerParams(const char* filename);

    // motor parameters
   unsigned short minDuty = 10;
   unsigned short maxDuty = 100;
   bool invertMotorPolarity = false;
   std::chrono::milliseconds stallCheckPeriod{1000};
   degrees stallThreshold = 0;
   unsigned short destallDuty = 0;
   std::chrono::milliseconds destallDuration{0};
   unsigned short destallTries = 0;

   // movement parameters
   CookedAngle parkPosition = CookedAngle(0);
   degrees accelAngle = 20.0;
   degrees tolerance = 0.1;

   // control loop parameters
   std::chrono::milliseconds loopDelay{10};
   enum class IndicatorStyle { Bar, Percent } indicatorStyle = IndicatorStyle::Bar;
};

enum class ReturnValue
{
  Success = 0,
  ConfigError = 1,
  HardwareError = 2,
  Stall = 3,
  SlewNotFinished = 4
};

class Controller
{
public:
   Controller(ControllerParams initialParams);

   // Methods for getting the current angle in various flavors.
   RawAngle getRawAngle() const;
   CookedAngle getCookedAngle() const;
   UserAngle getUserAngle() const;

   // This is what it's all about.
   ReturnValue slew(CookedAngle targetAngle);

private:
   enum class MotorStatus { Undetermined, OK, Stalled, WrongDirection };

   void beginMotorMonitoring(const CookedAngle currentAngle);
   MotorStatus checkMotor(const CookedAngle currentAngle,
                          const float wantedDirection);

   ControllerParams params;
   Motor* motor;
   Sensor* sensor;

   CookedAngle stallCheckAngle{0};
   std::chrono::steady_clock::time_point stallCheckTime;
};

#endif // CONTROLLER_H
