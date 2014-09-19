#include <iostream>
#include <cmath>
#include <thread>
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
   CookedAngle::setOrigin(RawAngle(config.lookup("angles.hardwareOrigin_raw")));
   CookedAngle::setInverted(config.lookup("sensor.invert"));
   UserAngle::setOrigin(CookedAngle(config.lookup("angles.userOrigin_cooked")));

   // minimum and maximum angles
   auto minimumSafeAngle =
      CookedAngle(config.lookup("angles.minSafeAngle_cooked"));
   auto maximumSafeAngle =
      CookedAngle(config.lookup("angles.maxSafeAngle_cooked"));
   CookedAngle::setSafeLimits(minimumSafeAngle, maximumSafeAngle);

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


RawAngle Controller::getRawAngle()
{
   return sensor->getRawAngle();
}


CookedAngle Controller::getCookedAngle()
{
   return CookedAngle(sensor->getRawAngle());
}


UserAngle Controller::getUserAngle()
{
   return UserAngle(CookedAngle(sensor->getRawAngle()));
}


void Controller::slew(CookedAngle targetAngle)
{
   const float dutySpan = params.maxDuty - params.minDuty;

   CookedAngle initialAngle = getCookedAngle();
   float direction = (targetAngle.val > initialAngle.val ? 1.0 : -1.0);

   if (direction > 0)
      motor->turnOnDirPositive();
   else
      motor->turnOnDirNegative();
   
   beginMotorMonitoring(initialAngle);
   while (true)
   {
      CookedAngle angle = getCookedAngle();
      std::cout << "angle " << UserAngle(angle).val << std::endl;
      degrees diffInitial = direction * (angle - initialAngle);
      degrees diffTarget = direction * (targetAngle - angle);
      
      if (diffTarget < params.tolerance)
      {
         motor->setPWM(0);
         break;
      }

      float dutyInitial = (diffInitial / params.accelAngle) * dutySpan + params.minDuty;
      float dutyTarget = ((diffTarget - params.tolerance) / params.accelAngle) * dutySpan + params.minDuty;
      int duty = round(std::fmin(dutyInitial, dutyTarget));
      if (duty < params.minDuty)
         duty = params.minDuty;
      else if (duty > params.maxDuty)
         duty = params.maxDuty;
      
      motor->setPWM(round(duty));

      MotorStatus status = checkMotor(angle, direction);
      if (status == MotorStatus::Stalled)
      {
         std::cerr << "Stall detected!";
         break;
      }
      else if (status == MotorStatus::WrongDirection)
      {
         std::cerr << "Motor turning in wrong direction!\n";
         break;
      }

      std::this_thread::sleep_for(params.loopDelay);
   }
   
   motor->turnOff();
}


void Controller::beginMotorMonitoring(CookedAngle currentAngle)
{
   stallCheckAngle = currentAngle;
   stallCheckTime = std::chrono::steady_clock::now();
}

Controller::MotorStatus Controller::checkMotor(CookedAngle currentAngle, float wantedDirection)
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
