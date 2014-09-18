#include <cmath>
#include "angles.h"

degrees mod360(degrees value)
{
   return value - 360.0 * std::floor(value / 360.0);
}


CookedAngle::CookedAngle(const RawAngle raw)
{
   val = mod360((inverted ? -1.0 : 1.0) * (linearize(raw.val) - offset));
}


CookedAngle::CookedAngle(const UserAngle user)
{
   val = user.val + UserAngle::userOrigin.val;
}


void CookedAngle::setLinearization(const std::vector<float>& coefficients)
{
   linCoeffs = coefficients;
   offset = linearize(hardwareOrigin.val);
}


void CookedAngle::setOrigin(const RawAngle origin)
{
   hardwareOrigin = origin;
   offset = linearize(hardwareOrigin.val);
}


void CookedAngle::setInverted(const bool set)
{
   inverted = set;
}


degrees CookedAngle::linearize(degrees val)
{
   float rad = val * M_PI / 180.0;
   for (unsigned int i = 0; i < linCoeffs.size(); i += 2)
   {
      float arg = (1 + i/2) * rad;
      val -= linCoeffs[i]   * cos(arg);
      val -= linCoeffs[i+1] * sin(arg);
   }
   return val;
}


UserAngle::UserAngle(const CookedAngle cooked)
{
   val = cooked.val - userOrigin.val;
}


void UserAngle::setOrigin(const CookedAngle origin)
{
   userOrigin = origin;
}

std::vector<float> CookedAngle::linCoeffs;
RawAngle CookedAngle::hardwareOrigin = RawAngle(0);
degrees CookedAngle::offset = 0;
bool CookedAngle::inverted = false;
CookedAngle UserAngle::userOrigin = CookedAngle(0);
