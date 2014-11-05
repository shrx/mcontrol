#ifndef SIMULATED_H
#define SIMULATED_H

#include <chrono>
#include <random>
#include "interface.h"

class SimulatedMotor : public Motor
{
public:
   SimulatedMotor(degrees relativeInitialAngle = 0);
   void turnOff();
   void setPWM(unsigned short duty);
   degrees currentAngle();
   void setVerbose(bool verbose);

private:
   void turnOnDir1();
   void turnOnDir2();
   void event();

   int engaged = 0;
   int duty = 0;
   bool initialStall;
   degrees internalAngle;
   std::chrono::steady_clock::time_point lastEvent;
   bool verbose = false;

   const unsigned short minimum_duty = 15;
   const unsigned short maximum_duty = 30;
   const degrees initialAngle = 250;
   const degrees minimum_angle = 230;
   const degrees maximum_angle = minimum_angle + 360 - 40;
   const float rpm_capability = 10.0 / 6.0;

   // stall simulation
   const unsigned short stall_overcome_duty = 0;
};

class SimulatedSensor : public Sensor
{
public:
   SimulatedSensor(SimulatedMotor* driver);
   RawAngle getRawAngle();

private:
   SimulatedMotor* motor;
   std::mt19937_64 generator;
   std::normal_distribution<degrees> normdist{0.0, 0.1};
   unsigned int numberOfReadouts = 0;

   const unsigned int randomSpikePeriod = 233;
   std::uniform_real_distribution<degrees> spikedist{0.0, 360.0};
};

#endif // SIMULATED_H
