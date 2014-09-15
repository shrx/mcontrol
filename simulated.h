#ifndef SIMULATED_H
#define SIMULATED_H

#include <chrono>
#include <random>
#include "interface.h"

class SimulatedMotor : public Motor
{
public:
   SimulatedMotor(float initialAngle = 0);
   void turnOff();
   void setPWM(unsigned short duty);
   float currentAngle();

private:
   void turnOnDir1();
   void turnOnDir2();
   void event();

   int rotating = 0;
   int duty = 0;
   float internalAngle;
   std::chrono::steady_clock::time_point lastEvent;
   
   const unsigned short minimum_duty = 10;
   const unsigned short maximum_duty = 100;
   const float minimum_angle = -20;
   const float maximum_angle = 380;
   const float rpm_capability = 10.0 / 6.0;
};

class SimulatedSensor : public Sensor
{
public:
   SimulatedSensor(SimulatedMotor* driver);
   float getRawAngle();

private:
   SimulatedMotor* motor;
   std::mt19937_64 generator;
   std::normal_distribution<float> normdist{0.0, 0.1};
};

#endif // SIMULATED_H
