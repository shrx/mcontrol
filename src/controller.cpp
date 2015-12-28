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
#include <cmath>
#include <thread>
#include <string>
#include <cstdio>
#include <atomic>
#include <csignal>
#include <list>
#include <algorithm>
#include <libconfig.h++>
#include "controller.h"

#ifdef HARDWARE
   #include <wiringPi.h>
   #include <wiringPiSPI.h>
   #include <linux/spi/spidev.h>
   #include "hardware.h"
#else
   #include "simulated.h"
#endif

ControllerParams::ControllerParams(const char* filename)
{
   libconfig::Config config;
   config.readFile(filename);
   config.setAutoConvert(true);

   // motor
   accelAngle = config.lookup("movement.accelAngle");
   minDuty = (unsigned int)config.lookup("motor.minDuty");
   maxDuty = (unsigned int)config.lookup("motor.maxDuty");
   invertMotorPolarity = config.lookup("motor.invertPolarity");
   stallCheckPeriod =
      std::chrono::milliseconds((unsigned int)config.lookup("motor.stallCheckPeriod"));
   stallThreshold = config.lookup("motor.stallThreshold");
   destallDuty = (unsigned int)config.lookup("motor.destallDuty");
   destallDuration =
      std::chrono::milliseconds((unsigned int)config.lookup("motor.destallDuration"));
   destallTries = (unsigned int)config.lookup("motor.destallTries");

   // angle conversions
   libconfig::Setting& linArray = config.lookup("angles.linearization");
   if (!linArray.isArray())
      throw ConfigFileException("angles.linearization must be an array");
   if (linArray.getLength() % 2)
      throw ConfigFileException("angles.linearization must contain an even number of values");

   std::vector<float> coeffs;
   for (int i = 0; i < linArray.getLength(); i++)
      coeffs.push_back(linArray[i]);

   CookedAngle::setLinearization(coeffs);

   /* Minimum and maximum raw angles.
    *
    * This also determines the orientation of the cooked angle scale: it is
    * established in such a direction that when going from the minimum angle
    * towards the maximum angle along the longest of the two possible paths,
    * the cooked angles increase in value.
   */
   auto rawAngleAtMinimum =
      RawAngle(config.lookup("angles.rawAngleAtMinimum"));
   auto rawAngleAtMaximum =
      RawAngle(config.lookup("angles.rawAngleAtMaximum"));
   degrees positiveRange = mod360(rawAngleAtMaximum.val - rawAngleAtMinimum.val);
   RawAngle halfRange(rawAngleAtMinimum + positiveRange/2.0);
   if (positiveRange >= 180)
   {
      CookedAngle::setOrigin(halfRange + 180.0);
      CookedAngle::setInverted(false);
   }
   else
   {
      CookedAngle::setOrigin(halfRange);
      CookedAngle::setInverted(true);
   }
   degrees endGuard = config.lookup("angles.endGuard");
   CookedAngle::setSafeLimits(CookedAngle(rawAngleAtMinimum) + endGuard,
                              CookedAngle(rawAngleAtMaximum) - endGuard);

   RawAngle userOriginPoint = RawAngle(config.lookup("angles.userOriginPoint"));
   degrees userOriginValue = config.lookup("angles.userOriginValue");
   UserAngle::setOrigin(CookedAngle(userOriginPoint) - userOriginValue);
   parkPosition = CookedAngle(RawAngle(config.lookup("movement.rawParkPosition")));
   if (!parkPosition.isSafe())
      throw ConfigFileException("park position is not within safe limits - please recheck");

   // control loop parameters
   tolerance = config.lookup("movement.tolerance");
   loopDelay = std::chrono::milliseconds(10);
}


Controller::Controller(ControllerParams initialParams) :
   params(initialParams)
{
   // setup
#ifdef HARDWARE
   if (wiringPiSetup() == -1)
   {
      perror("wiringPiSetup");
      exit(2);
   }

   int fd = wiringPiSPISetupMode(0, 500000, SPI_MODE_1);
   if (fd == -1)
   {
      perror("wiringPiSPISetupMode");
      exit(2);
   }

   // Eeek! Hardcoded magic numbers!!
   // Seriously, if you came so far as to need this program, you are more
   // than well equipped to know what to set these to.
   motor = new HardwareMotor(4, 5, 1);
   sensor = new HardwareSensor;
#else
   motor = new SimulatedMotor(30);
   sensor = new SimulatedSensor(dynamic_cast<SimulatedMotor*>(motor));
#endif

   motor->invertPolarity(params.invertMotorPolarity);
}


RawAngle Controller::getRawAngle() const
{
   return sensor->getRawAngle();
}


CookedAngle Controller::getCookedAngle() const
{
   std::list<CookedAngle> readouts;
   // This parameter was deemed too obscure to be put in the config file.
   const unsigned int numberOfReadouts = 5;

   // Read a few consecutive values from the sensor.
   for (unsigned int i = 0; i < numberOfReadouts; i++)
      readouts.emplace_back(sensor->getRawAngle());

   // Get rid of the minimum and maximum value, hopefully throwing out any
   // erroneous readings.
   readouts.erase(std::min_element(readouts.begin(), readouts.end()));
   readouts.erase(std::max_element(readouts.begin(), readouts.end()));

   // Of the remaining values, return the most recent one.
   return readouts.back();
}


UserAngle Controller::getUserAngle() const
{
   return UserAngle(getCookedAngle());
}


/* An abstract progress indicator. It provides the core of a progress indicator
 * that prints the current state at predetermined time intervals.
*/
class ProgressIndicator
{
public:
   ProgressIndicator(CookedAngle initial_, CookedAngle target_) :
      initial(0), target(0)
   {
      reset(initial_, target_);
   }

   virtual ~ProgressIndicator() = default;

   // Print the current progress if either enough time has elapsed from the
   // previous printing or the forcePrint parameter is true.
   void print(CookedAngle angle, bool forcePrint = false)
   {
      auto now = std::chrono::steady_clock::now();
      if (!forcePrint && now < previousPrint + printPeriod)
         return;
      previousPrint = now;
      printProgress(angle);
   }

   // Set new initial and target angles.
   void reset(CookedAngle initial_, CookedAngle target_)
   {
      initial = initial_;
      target = target_;
      previousPrint = std::chrono::steady_clock::now() - printPeriod;
   }

   // Finalize the output (for example, by printing a final newline).
   virtual void finalize() = 0;

protected:
   // Do the actual printing.
   virtual void printProgress(CookedAngle angle) = 0;

   CookedAngle initial;
   CookedAngle target;
   std::chrono::steady_clock::time_point previousPrint;

   static const int length = 30;
   static constexpr std::chrono::milliseconds printPeriod{100};
};

// definitions for the above static consts
const int ProgressIndicator::length;
constexpr std::chrono::milliseconds ProgressIndicator::printPeriod;


// An ASCII progress bar.
class BarIndicator : public ProgressIndicator
{
public:
   BarIndicator(CookedAngle initial_, CookedAngle target_) :
      ProgressIndicator(initial_, target_) {}

   virtual void finalize()
   {
      std::cout << std::endl;
   }

private:
   virtual void printProgress(CookedAngle angle)
   {
      std::string bar(length, '-');
      int position = std::round(length * (angle - initial)/(target - initial));
      position = std::min(std::max( position, 0), length - 1);
      bar.replace (0,  position,  position, '=');
      bar[position] = '>';
      printf("\r\033[K%6.1f degrees %s", UserAngle(angle).val, bar.c_str());
      fflush(stdout);
   }

   // Length of the bar in characters.
   static const int length = 30;
};


/* An indicator with simple numeric output suitable for further processing.
 * It outputs lines with the format:
 *
 * <current angle> <progress percent>
*/
class PercentIndicator : public ProgressIndicator
{
public:
   PercentIndicator(CookedAngle initial_, CookedAngle target_) :
      ProgressIndicator(initial_, target_) {}

   virtual void finalize() {}

private:
   virtual void printProgress(CookedAngle angle)
   {
      printf("%.1f %d\n",
             UserAngle(angle).val,
             (int)std::round(100 * (angle - initial)/(target - initial)));
      fflush(stdout);
   }
};


// The number of SIGINTs (Ctrl+C) received during the slew.
// Helps us to estimate the user's panic level and act accordingly. :-)
std::atomic_int timesInterrupted(0);

// SIGINT handler
void int_handler(int sig)
{
   timesInterrupted++;
   signal(SIGINT, int_handler);
}


/*****************************
**** THE MEAT OF THE STUFF ***
******************************/

ReturnValue Controller::slew(CookedAngle targetAngle)
{
   ReturnValue retval = ReturnValue::Success;

   enum class SlewPhase
   {
      accelerating,
      plateau,
      decelerating
   };

   SlewPhase phase = SlewPhase::accelerating;
   int interruptsHandled = 0;
   signal(SIGINT, int_handler);

   // Determine which direction to turn and enage the H-bridge accordingly.
   CookedAngle initialAngle = getCookedAngle();
   float direction = (targetAngle.val > initialAngle.val ? 1.0 : -1.0);

   if (direction > 0)
      motor->turnOnDirPositive();
   else
      motor->turnOnDirNegative();

   // Create a progress indicator.
   ProgressIndicator* progressIndicator;
   if (params.indicatorStyle == ControllerParams::IndicatorStyle::Bar)
      progressIndicator = new BarIndicator(initialAngle, targetAngle);
   else
      progressIndicator = new PercentIndicator(initialAngle, targetAngle);

   // Start motor monitoring. This will take a record of the angle just before
   // we apply power to the motor.
   beginMotorMonitoring(initialAngle);
   int initialStallsPermitted = params.destallTries;

   // Main control loop.
   while (true)
   {
      CookedAngle angle = getCookedAngle();
      degrees diffInitial = direction * (angle - initialAngle);
      degrees diffTarget = direction * (targetAngle - angle);
      progressIndicator->print(angle);

      if (diffTarget < params.tolerance)
      {
         // We are done. Force printing of the final angle value.
         progressIndicator->print(angle, true);
         break;
      }

      // Now determine the slew phase that we are in and the needed PWM duty
      // cycle. We do this by calculating the duties for both accelerating and
      // decelerating and then taking the lower one.
      const float dutySpan = params.maxDuty - params.minDuty;
      float dutyInitial = (diffInitial / params.accelAngle) * dutySpan + params.minDuty;
      float dutyTarget = ((diffTarget - params.tolerance) / params.accelAngle) * dutySpan + params.minDuty;
      int duty;

      if (dutyInitial <= dutyTarget)
      {
         phase = SlewPhase::accelerating;
         duty = round(dutyInitial);
      }
      else
      {
         phase = SlewPhase::decelerating;
         duty = round(dutyTarget);
      }

      // Clamp the duty cycle if it is outside the wanted range.
      if (duty < params.minDuty)
         duty = params.minDuty;
      else if (duty >= params.maxDuty)
      {
         // Notice that for short slews, there can be no plateau.
         phase = SlewPhase::plateau;
         duty = params.maxDuty;
      }

      motor->setPWM(duty);

      // Check on what the axis is actually doing.
      MotorStatus status = checkMotor(angle, direction);
      if (status == MotorStatus::Stalled)
      {
         if (initialStallsPermitted > 0)
         {
            int destallTry = params.destallTries - initialStallsPermitted + 1;
            std::cerr << "\nInitial stall detected. Performing a de-stall maneuver "
                      << destallTry << "/" << params.destallTries << ".\n";
            motor->setPWM(params.destallDuty);
            std::this_thread::sleep_for(params.destallDuration);
            motor->setPWM(duty);
            initialStallsPermitted--;
         }
         else
         {
            std::cerr << "\nStall detected!";
            retval = ReturnValue::Stall;
            break;
         }
      }
      else if (status == MotorStatus::WrongDirection)
      {
         std::cerr << "\nMotor turning in wrong direction!";
         retval = ReturnValue::HardwareError;
         break;
      }
      else if (status == MotorStatus::OK)
      {
         // We are now certain that the motor is moving. If a stall occurs from
         // now on, it is certainly not an initial stall.
         initialStallsPermitted = 0;
      }

      // Check if the user's panic level has increased recently.
      if (timesInterrupted > interruptsHandled )
      {
         interruptsHandled = timesInterrupted;
         if (interruptsHandled == 1)
         {
            std::cerr << "\nInterrupted, stopping gracefully. Give Ctrl+C again for immediate stop.\n";
            retval = ReturnValue::SlewNotFinished;

            // Determine the closest target angle that we can reach by slowly
            // decelerating.
            if (phase == SlewPhase::accelerating)
               targetAngle = angle + direction * diffInitial;
            else if (phase == SlewPhase::plateau)
               targetAngle = angle + direction * params.accelAngle;
            // else if (phase == SlewPhase::decelerating)
            //    Already decelerating - nothing to do.

            // From now on, the progress bar shows the progress of stopping.
            progressIndicator->reset(angle, targetAngle);
         }
         else
         {
            std::cerr << "\nEmergency stop. Hold on to your gears!";
            retval = ReturnValue::SlewNotFinished;
            break;
         }
      }
      std::this_thread::sleep_for(params.loopDelay);
   }
   progressIndicator->finalize();
   delete progressIndicator;

   // De-energize the motor and turn off the H-bridge switches.
   motor->setPWM(0);
   motor->turnOff();
   signal(SIGINT, SIG_DFL);
   return retval;
}


/* Records the current angle and the timestamp. This will later be used to tell
 * if the motor is spinning or not.
*/
void Controller::beginMotorMonitoring(const CookedAngle currentAngle)
{
   stallCheckAngle = currentAngle;
   stallCheckTime = std::chrono::steady_clock::now();
}


/* Checks if the angle readings are going in the direction that we expect them
 * to go. If not, determines whether the motor is not moving at all or it is
 * spinning in the wrong direction.
*/
Controller::MotorStatus Controller::checkMotor(const CookedAngle currentAngle,
                                               const float wantedDirection)
{
   MotorStatus status = MotorStatus::Undetermined;

   auto currentTime = std::chrono::steady_clock::now();
   if (currentTime >= stallCheckTime + params.stallCheckPeriod)
   {
      auto difference = currentAngle - stallCheckAngle;
      if (abs(difference) < params.stallThreshold)
      {
         // No relevant change from when we last looked. This is a stall.
         status = MotorStatus::Stalled;
      }
      else if (std::signbit(difference) != std::signbit(wantedDirection))
      {
         // The axis is spinning, but in the wrong direction.
         status = MotorStatus::WrongDirection;
      }
      else
         status = MotorStatus::OK;

      // Record the current state for later.
      stallCheckAngle = currentAngle;
      stallCheckTime = currentTime;
   }
   return status;
}
