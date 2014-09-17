#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <chrono>
#include "angles.h"
#include "interface.h"

struct ControllerParams
{
    // motor parameters
   degrees accelAngle = 20.0;
   unsigned short minDuty = 10;
   unsigned short maxDuty = 100;
   bool invertMotorPolarity = false;
   std::chrono::milliseconds stallCheckPeriod{1000};
   degrees stallThreshold = 0;

   // angle parameters
   CookedAngle minimumSafeAngle{0};
   CookedAngle maximumSafeAngle{360};
   degrees tolerance = 0.1;
   
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
   enum class MotorStatus { OK, Stalled, WrongDirection };

   void beginMotorMonitoring(CookedAngle currentAngle);
   MotorStatus checkMotor(CookedAngle currentAngle, float wantedDirection);

   ControllerParams params;
   Motor* motor;
   Sensor* sensor;

   CookedAngle stallCheckAngle{0};
   std::chrono::steady_clock::time_point stallCheckTime;
};

#endif // CONTROLLER_H
