#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <chrono>
#include "angles.h"
#include "interface.h"

struct ControllerParams
{
    // motor parameters
   float accelAngle = 20.0;
   unsigned short minDuty = 0;
   unsigned short maxDuty = 100;
   bool invertMotorPolarity = false;

   // angle parameters
   CookedAngle minimumSafeAngle{0};
   CookedAngle maximumSafeAngle{360};
   float tolerance = 0.1;
   
   // control loop parameters
   std::chrono::milliseconds loopDelay{10};
};


class Controller
{
public:
   Controller(ControllerParams initialParams);
   RawAngle getRawAngle();
   CookedAngle getCookedAngle();
   UserAngle getUserAngle();
   bool isValidAngle(CookedAngle);
   bool isValidAngle(UserAngle);
   void slew(CookedAngle targetAngle);

private:
   ControllerParams params;
   Motor* motor;
   Sensor* sensor;
};

#endif // CONTROLLER_H
