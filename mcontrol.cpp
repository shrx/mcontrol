#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <tclap/CmdLine.h>
#include "simulated.h"

#ifdef HARDWARE
#include <wiringPi.h>
#include "hardware.h"
#endif


// params
const float accelAngle = 20.0;
const unsigned short minDuty = 15;
const unsigned short maxDuty = 30;
const bool invertMotorPolarity = false;
const bool invertSystemPolarity = false;
const float zeroPosition = 0;
const float tolerance = 0.1;
auto loopDelay = std::chrono::milliseconds(10);

// Motor and sensor
Motor* motor;
Sensor* sensor;


void slew(float targetAngle)
{
   const float dutySpan = maxDuty - minDuty;

   float initialAngle = sensor->getCookedAngle();
   float sense = (targetAngle > initialAngle ? 1.0 : -1.0);

   if (sense > 0)
      motor->turnOnDirPositive();
   else
      motor->turnOnDirNegative();
   
   while (true)
   {
      float angle = sensor->getCookedAngle();
      std::cout << "angle " << angle << std::endl;
      float diffInitial = sense * (angle - initialAngle);
      float diffTarget = sense * (targetAngle - angle);
      
      if (diffTarget < tolerance)
      {
         motor->setPWM(0);
         break;
      }

      float dutyInitial = (diffInitial / accelAngle) * dutySpan + minDuty;
      float dutyTarget = ((diffTarget - tolerance) / accelAngle) * dutySpan + minDuty;
      int duty = round(std::fmin(dutyInitial, dutyTarget));
      if (duty < minDuty)
         duty = minDuty;
      else if (duty > maxDuty)
         duty = maxDuty;
      
      motor->setPWM(round(duty));
      std::this_thread::sleep_for(loopDelay);
   }
   
   motor->turnOff();
}

class TCLAPangleConstraint : public TCLAP::Constraint<float>
{
public:
   std::string description() const { return "0 <= angle < 360"; }
   std::string shortID() const { return "angle"; }
   bool check(const float& value) const { return (value >= 0 && value < 360); }
} angleConstraint;


int main(int argc, char *argv[])
{
   TCLAP::CmdLine cmd("Motor control");
   
   TCLAP::SwitchArg arg_queryAngle("q", "query-angle", "Query angle");
   TCLAP::SwitchArg arg_queryRawAngle("r", "raw-angle", "Query raw angle");
   TCLAP::UnlabeledValueArg<float> arg_targetAngle(
      "angle", "Slew to this angle", false, 0, &angleConstraint);
   
   auto xorArgs = std::vector<TCLAP::Arg*>{
      &arg_queryAngle,
      &arg_queryRawAngle,
      &arg_targetAngle};

   cmd.xorAdd(xorArgs);
   cmd.parse(argc, argv);
   
   // setup
#ifdef HARDWARE
   wiringPiSetup();
   // TODO: and possibly other stuff required for wiringPi
   
   motor = new HardwareMotor(1, 2, 3); // TODO: pin numbers
   sensor = new HardwareSensor;
#else
   motor = new SimulatedMotor(50);
   sensor = new SimulatedSensor(dynamic_cast<SimulatedMotor*>(motor));
#endif

   motor->invertPolarity(invertMotorPolarity ^ invertSystemPolarity);
   sensor->invertPolarity(invertSystemPolarity);
   sensor->setZeroPosition(zeroPosition);

   if (arg_targetAngle.isSet())
      slew(arg_targetAngle.getValue());
   else
   {
      float angle;
      if (arg_queryRawAngle.isSet())
         angle = sensor->getRawAngle();
      else
         angle = sensor->getCookedAngle();
      
      std::cout << angle << std::endl;
   }

   return 0;
}
