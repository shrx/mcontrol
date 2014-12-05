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

#include <iostream>
#include <ratio>
#include "simulated.h"
#include "angles.h"

SimulatedMotor::SimulatedMotor(degrees relativeInitialAngle)
{
  internalAngle = initialAngle + relativeInitialAngle;
  initialStall = true;
}

void SimulatedMotor::turnOnDir1()
{
   event();
   engaged = 1;
   if (verbose)
     std::cerr << "motor: Dir1\n";
}

void SimulatedMotor::turnOnDir2()
{
   event();
   engaged = -1;
   if (verbose)
      std::cerr << "motor: Dir2\n";
}

void SimulatedMotor::turnOff()
{
   event();
   engaged = 0;
   initialStall = true;
   if (verbose)
      std::cerr << "motor: off\n";
}

/* This class monitors a condition and prints a message when the condition
 * changes from true to false. Useful for pointing out the exact moment at
 * which an assertion is first violated, but keeping silent at all other times.
*/
class assertTrigger
{
public:
   bool operator()(const bool state)
   {
      bool result = (oldState ? false : state);
      oldState = state;
      return result;
   }

   void operator()(const bool state, const std::string& message)
   {
      if (operator()(state))
         std::cout << message;
   }

private:
   bool oldState = false;
};

void SimulatedMotor::setPWM(unsigned short duty)
{
   {
      // Warn the user if the duty cycle exceeds the safe limit.
      static assertTrigger t;
      if (t(duty > maximum_duty))
         std::cerr << "motor: ERROR: duty cycle exceeds maximum ("
                   << duty << " > " << maximum_duty << ")\n";
   }

   if (duty > maximum_duty)
      duty = maximum_duty;

   event();
   this->duty = duty;

   if (verbose)
      std::cerr << "motor: PWM set to " << duty << "\n";

   {
      // Report a stall when the duty cycle is too low.
      static assertTrigger t;
      t(duty > 0 && duty < minimum_duty, "motor: WARNING: stalled!\n");
   }
}

void SimulatedMotor::event()
{
   // Check if we are in an initial stall and if yes, whether the current PWM
   // duty cycle is large enough to put the motor out of it.
   if (engaged && initialStall && (duty >= stall_overcome_duty))
      initialStall = false;

   if (engaged && !initialStall)
   {
      // Normal operation. Update the position according to the direction of
      // spinning and the time elapsed since the previous event.
      using std::chrono::duration_cast;
      using std::chrono::duration;
      using std::ratio;

      float effectiveDuty = (duty < minimum_duty ? 0 : duty) / 100.0;
      float rpm = rpm_capability * effectiveDuty;
      auto currentTime = std::chrono::steady_clock::now();
      auto elapsedMin = duration_cast<duration<float,ratio<60,1>>>(currentTime - lastEvent).count();
      internalAngle += 360.0 * (rpm * elapsedMin * engaged);

      {
         // Warn if the motor reached the lower end switch.
         static assertTrigger t;
         if (t(internalAngle < minimum_angle))
         {
            std::cerr << "motor: WARNING: safety switch engaged @ mininum ("
                      << internalAngle << " < " << minimum_angle << ")\n";
         }
      }
      // Do not allow rotating past the end switch.
      if (internalAngle < minimum_angle)
         internalAngle = minimum_angle;

      {
         // Warn if the motor reached the upper end switch.
         static assertTrigger t;
         if (t(internalAngle > maximum_angle))
         {
            std::cerr << "motor: WARNING: safety switch engaged @ maximum ("
                      << internalAngle << " > " << maximum_angle << ")\n";
         }
      }
      // Do not allow rotating past the end switch.
      if (internalAngle > maximum_angle)
         internalAngle = maximum_angle;
   }

   lastEvent = std::chrono::steady_clock::now();
}

degrees SimulatedMotor::currentAngle()
{
   event();
   return internalAngle;
}

void SimulatedMotor::setVerbose(const bool verbose_)
{
   verbose = verbose_;
}


SimulatedSensor::SimulatedSensor(SimulatedMotor* driver) : motor(driver)
{}

RawAngle SimulatedSensor::getRawAngle()
{
   numberOfReadouts++;
   if (randomSpikePeriod && (numberOfReadouts % randomSpikePeriod == 0))
   {
      // Return a completely random value every now and then.
      return RawAngle(spikedist(generator));
   }

   return RawAngle(mod360(motor->currentAngle() + normdist(generator)));
}
