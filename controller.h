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

   // movement parameters
   degrees accelAngle = 20.0;
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
