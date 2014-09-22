#include <iostream>
#include <cmath>
#include <thread>
#include <string>
#include <cstdio>
#include <atomic>
#include <csignal>
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
   return CookedAngle(sensor->getRawAngle());
}


UserAngle Controller::getUserAngle() const
{
   return UserAngle(CookedAngle(sensor->getRawAngle()));
}


class ProgressBar
{
public:
   ProgressBar(CookedAngle initial_, CookedAngle target_) :
      initial(initial_), target(target_),
      previousPrint(std::chrono::steady_clock::now() - printPeriod)
      {}

   void print(CookedAngle angle, bool forcePrint = false)
   {
      auto now = std::chrono::steady_clock::now();
      if (!forcePrint && now < previousPrint + printPeriod)
         return;
      previousPrint = now;

      std::string bar(length, '-');
      int position = std::round(length * (angle - initial)/(target - initial));
      position = std::min(std::max( position, 0), length - 1);
      bar.replace (0,  position,  position, '=');
      bar[position] = '>';
      printf("\r\033[K%6.1f degrees %s", UserAngle(angle).val, bar.c_str());
      fflush(stdout);
   }

private:
   CookedAngle initial;
   CookedAngle target;
   std::chrono::steady_clock::time_point previousPrint;
   static const int length = 30;
   static constexpr std::chrono::milliseconds printPeriod{100};
};

// definitions for the above static consts
const int ProgressBar::length;
constexpr std::chrono::milliseconds ProgressBar::printPeriod;


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

   beginMotorMonitoring(initialAngle);
   ProgressBar progressBar(initialAngle, targetAngle);
   while (true)
   {
      CookedAngle angle = getCookedAngle();
      degrees diffInitial = direction * (angle - initialAngle);
      degrees diffTarget = direction * (targetAngle - angle);
      progressBar.print(angle);

      if (diffTarget < params.tolerance)
      {
         // Force printing of the final angle value.
         progressBar.print(angle, true);
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
         std::cerr << "\nStall detected!\n";
         break;
      }
      else if (status == MotorStatus::WrongDirection)
      {
         std::cerr << "\nMotor turning in wrong direction!\n";
         break;
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
            progressBar = ProgressBar(angle, targetAngle);
         }
         else
         {
            std::cerr << "\nEmergency stop. Hold on to your gears!";
            break;
         }
      }
      std::this_thread::sleep_for(params.loopDelay);
   }
   std::cout << std::endl;

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
   MotorStatus status = MotorStatus::OK;

   auto currentTime = std::chrono::steady_clock::now();
   if (currentTime >= stallCheckTime + params.stallCheckPeriod)
   {
      auto difference = currentAngle - stallCheckAngle;
      if (abs(difference) < params.stallThreshold)
         status = MotorStatus::Stalled;
      else if (std::signbit(difference) != std::signbit(wantedDirection))
         status = MotorStatus::WrongDirection;

      stallCheckAngle = currentAngle;
      stallCheckTime = currentTime;
   }
   return status;
}
