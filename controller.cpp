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

   // Minimum and maximum raw angles.
   // This also determines the orientation of the angle scale.
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

   int fd = wiringPiSPISetupWithMode(0, 500000, SPI_MODE_1);
   if (fd == -1)
   {
      perror("wiringPiSPISetupWithMode");
      exit(2);
   }

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


class ProgressIndicator
{
public:
   ProgressIndicator(CookedAngle initial_, CookedAngle target_) :
      initial(0), target(0)
   {
      reset(initial_, target_);
   }

   virtual ~ProgressIndicator() = default;

   void print(CookedAngle angle, bool forcePrint = false)
   {
      auto now = std::chrono::steady_clock::now();
      if (!forcePrint && now < previousPrint + printPeriod)
         return;
      previousPrint = now;
      printProgress(angle);
   }

   void reset(CookedAngle initial_, CookedAngle target_)
   {
      initial = initial_;
      target = target_;
      previousPrint = std::chrono::steady_clock::now() - printPeriod;
   }

   virtual void finalize() = 0;

protected:
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

   static const int length = 30;
};


class PercentIndicator : public ProgressIndicator
{
public:
   PercentIndicator(CookedAngle initial_, CookedAngle target_) :
      ProgressIndicator(initial_, target_) {}

   virtual void finalize() {}

private:
   virtual void printProgress(CookedAngle angle)
   {
      printf("%d\n", (int)std::round(100 * (angle - initial)/(target - initial)));
   }
};


std::atomic_int timesInterrupted(0);

// SIGINT handler
void int_handler(int sig)
{
   timesInterrupted++;
   signal(SIGINT, int_handler);
}

void Controller::slew(CookedAngle targetAngle)
{
   enum class SlewPhase
   {
      accelerating,
      plateau,
      decelerating
   };

   SlewPhase phase = SlewPhase::accelerating;
   int interrupts = 0;
   signal(SIGINT, int_handler);

   CookedAngle initialAngle = getCookedAngle();
   float direction = (targetAngle.val > initialAngle.val ? 1.0 : -1.0);

   if (direction > 0)
      motor->turnOnDirPositive();
   else
      motor->turnOnDirNegative();

   ProgressIndicator* progressIndicator;
   if (params.indicatorStyle == ControllerParams::IndicatorStyle::Bar)
      progressIndicator = new BarIndicator(initialAngle, targetAngle);
   else
      progressIndicator = new PercentIndicator(initialAngle, targetAngle);

   beginMotorMonitoring(initialAngle);
   int initialStallsPermitted = (params.destallDuty > 0 ? 1 : 0);
   while (true)
   {
      CookedAngle angle = getCookedAngle();
      degrees diffInitial = direction * (angle - initialAngle);
      degrees diffTarget = direction * (targetAngle - angle);
      progressIndicator->print(angle);

      if (diffTarget < params.tolerance)
      {
         // Force printing of the final angle value.
         progressIndicator->print(angle, true);
         break;
      }

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

      if (duty < params.minDuty)
         duty = params.minDuty;
      else if (duty >= params.maxDuty)
      {
         phase = SlewPhase::plateau;
         duty = params.maxDuty;
      }

      motor->setPWM(duty);

      MotorStatus status = checkMotor(angle, direction);
      if (status == MotorStatus::Stalled)
      {
         if (initialStallsPermitted > 0)
         {
            std::cerr << "\nInitial stall detected. Performing a de-stall maneuver.\n";
            motor->setPWM(params.destallDuty);
            std::this_thread::sleep_for(params.destallDuration);
            motor->setPWM(duty);
            initialStallsPermitted--;
         }
         else
         {
            std::cerr << "\nStall detected!";
            break;
         }
      }
      else if (status == MotorStatus::WrongDirection)
      {
         std::cerr << "\nMotor turning in wrong direction!";
         break;
      }
      else if (status == MotorStatus::OK)
      {
         // We are now certain that the motor is moving. If a stall occurs from
         // now on, it is certainly not an initial stall.
         initialStallsPermitted = 0;
      }

      if (timesInterrupted > interrupts )
      {
         interrupts = timesInterrupted;
         if (interrupts == 1)
         {
            std::cerr << "\nInterrupted, stopping gracefully. Give Ctrl+C again for immediate stop.\n";

            if (phase == SlewPhase::accelerating)
               targetAngle = angle + direction * diffInitial;
            else if (phase == SlewPhase::plateau)
               targetAngle = angle + direction * params.accelAngle;
            // else if (phase == SlewPhase::decelerating)
            //    Already decelerating - nothing to do.

            // From now on, the progress bar only shows the progress of stopping.
            progressIndicator->reset(angle, targetAngle);
         }
         else
         {
            std::cerr << "\nEmergency stop. Hold on to your gears!";
            break;
         }
      }
      std::this_thread::sleep_for(params.loopDelay);
   }
   progressIndicator->finalize();
   delete progressIndicator;

   motor->setPWM(0);
   motor->turnOff();
   signal(SIGINT, SIG_DFL);
}


void Controller::beginMotorMonitoring(const CookedAngle currentAngle)
{
   stallCheckAngle = currentAngle;
   stallCheckTime = std::chrono::steady_clock::now();
}

Controller::MotorStatus Controller::checkMotor(const CookedAngle currentAngle,
                                               const float wantedDirection)
{
   MotorStatus status = MotorStatus::Undetermined;

   auto currentTime = std::chrono::steady_clock::now();
   if (currentTime >= stallCheckTime + params.stallCheckPeriod)
   {
      auto difference = currentAngle - stallCheckAngle;
      if (abs(difference) < params.stallThreshold)
         status = MotorStatus::Stalled;
      else if (std::signbit(difference) != std::signbit(wantedDirection))
         status = MotorStatus::WrongDirection;
      else
         status = MotorStatus::OK;

      stallCheckAngle = currentAngle;
      stallCheckTime = currentTime;
   }
   return status;
}
