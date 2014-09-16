#include <iostream>
#include <cmath>
#include <thread>
#include "controller.h"

Controller::Controller(ControllerParams initialParams) :
   params(initialParams)
{
   // setup
#ifdef HARDWARE
   wiringPiSetup();
   // TODO: and possibly other stuff required for wiringPi
   
   motor = new HardwareMotor(1, 2, 3); // TODO: pin numbers
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


bool Controller::isValidAngle(CookedAngle angle)
{
   return (angle >= params.minimumSafeAngle) && (angle <= params.maximumSafeAngle);
}


bool Controller::isValidAngle(UserAngle angle)
{
   return isValidAngle(CookedAngle(angle));
}


void Controller::slew(CookedAngle targetAngle)
{
   const float dutySpan = params.maxDuty - params.minDuty;

   CookedAngle initialAngle = getCookedAngle();
   float sense = (targetAngle.val > initialAngle.val ? 1.0 : -1.0);

   if (sense > 0)
      motor->turnOnDirPositive();
   else
      motor->turnOnDirNegative();
   
   while (true)
   {
      CookedAngle angle = getCookedAngle();
      std::cout << "angle " << UserAngle(angle).val << std::endl;
      float diffInitial = sense * (angle - initialAngle);
      float diffTarget = sense * (targetAngle - angle);
      
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
      std::this_thread::sleep_for(params.loopDelay);
   }
   
   motor->turnOff();
}
