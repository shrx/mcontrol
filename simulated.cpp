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
   if (verbose)
     std::cerr << "motor: Dir1\n";
}

void SimulatedMotor::turnOnDir2()
{
   event();
   rotating = -1;
   if (verbose)
      std::cerr << "motor: Dir2\n";
}

void SimulatedMotor::turnOff()
{
   event();
   rotating = 0;
   if (verbose)
      std::cerr << "motor: off\n";
}

class assertTrigger
{
public:
   bool operator()(const bool state)
   {
      bool result = (oldState ? false : state);
      oldState = state;
      return result;
   }

   void operator()(const bool state, const std::string& message)
   {
      if (operator()(state))
         std::cout << message;
   }

private:
   bool oldState = false;
};

void SimulatedMotor::setPWM(unsigned short duty)
{
   {
      static assertTrigger t;
      if (t(duty > maximum_duty))
         std::cerr << "motor: ERROR: duty cycle exceeds maximum ("
                   << duty << " > " << maximum_duty << ")\n";
   }

   if (duty > maximum_duty)
      duty = maximum_duty;

   event();
   this->duty = duty;

   if (verbose)
      std::cerr << "motor: PWM set to " << duty << "\n";

   {
      static assertTrigger t;
      t(duty > 0 && duty < minimum_duty, "motor: WARNING: stalled!\n");
   }
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
      
      {
         static assertTrigger t;
         if (t(internalAngle < minimum_angle))
         {
            std::cerr << "motor: WARNING: safety switch engaged @ mininum ("
                      << internalAngle << " < " << minimum_angle << ")\n";
         }
      }
      if (internalAngle < minimum_angle)
         internalAngle = minimum_angle;

      {
      static assertTrigger t;
         if (t(internalAngle > maximum_angle))
         {
            std::cerr << "motor: WARNING: safety switch engaged @ maximum ("
                      << internalAngle << " > " << maximum_angle << ")\n";
         }
      }
      if (internalAngle > maximum_angle)
         internalAngle = maximum_angle;
   }
   
   lastEvent = std::chrono::steady_clock::now();
}

degrees SimulatedMotor::currentAngle()
{
   event();
   return internalAngle;
}

void SimulatedMotor::setVerbose(const bool verbose_)
{
   verbose = verbose_;
}


SimulatedSensor::SimulatedSensor(SimulatedMotor* driver) : motor(driver)
{}

RawAngle SimulatedSensor::getRawAngle()
{
   return RawAngle(mod360(motor->currentAngle() + normdist(generator)));
}
