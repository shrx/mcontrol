#include <iostream>
#include <ratio>
#include "simulated.h"
#include "angles.h"

SimulatedMotor::SimulatedMotor(degrees relativeInitialAngle)
{
  internalAngle = initialAngle + relativeInitialAngle;
}

void SimulatedMotor::turnOnDir1()
{
   event();
   rotating = 1;
   std::cerr << "motor: Dir1\n";
}

void SimulatedMotor::turnOnDir2()
{
   event();
   rotating = -1;
   std::cerr << "motor: Dir2\n";
}

void SimulatedMotor::turnOff()
{
   event();
   rotating = 0;
   std::cerr << "motor: off\n";
}

void SimulatedMotor::setPWM(unsigned short duty)
{
   if (duty > maximum_duty)
   {
      std::cerr << "motor: ERROR: duty cycle exceeds maximum ("
                << duty << " > " << maximum_duty << ")\n";
      duty = maximum_duty;
   }
   event();
   this->duty = duty;

   std::cerr << "motor: PWM set to " << duty << "\n";
   if (duty > 0 && duty < minimum_duty)
      std::cerr << "motor: WARNING: stalled!";
}

void SimulatedMotor::event()
{
   if (rotating)
   {
      using std::chrono::duration_cast;
      using std::chrono::duration;
      using std::ratio;

      float effectiveDuty = (duty < minimum_duty ? 0 : duty) / 100.0;
      float rpm = rpm_capability * effectiveDuty;
      auto currentTime = std::chrono::steady_clock::now();
      auto elapsedMin = duration_cast<duration<float,ratio<60,1>>>(currentTime - lastEvent).count();
      internalAngle += 360.0 * (rpm * elapsedMin * rotating);
      
      if (internalAngle < minimum_angle)
      {
         std::cerr << "motor: WARNING: safety switch engaged @ mininum ("
                   << internalAngle << " < " << minimum_angle << ")\n";
         internalAngle = minimum_angle;
      }

      if (internalAngle > maximum_angle)
      {
         std::cerr << "motor: WARNING: safety switch engaged @ maximum ("
                   << internalAngle << " > " << maximum_angle << ")\n";
         internalAngle = maximum_angle;
      }
   }
   
   lastEvent = std::chrono::steady_clock::now();
}

degrees SimulatedMotor::currentAngle()
{
   event();
   return internalAngle;
}


SimulatedSensor::SimulatedSensor(SimulatedMotor* driver) : motor(driver)
{}

RawAngle SimulatedSensor::getRawAngle()
{
   return RawAngle(mod360(motor->currentAngle() + normdist(generator)));
}
