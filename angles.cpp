#include <cmath>
#include "angles.h"

degrees mod360(degrees value)
{
   return value - 360.0 * std::floor(value / 360.0);
}

CookedAngle::CookedAngle(const RawAngle raw)
{
   // TODO: linearization
   val = mod360((inverted ? -1.0 : 1.0) * (raw.val - hardwareOrigin.val));
}

CookedAngle::CookedAngle(const UserAngle user)
{
   val = user.val + UserAngle::userOrigin.val;
}

UserAngle::UserAngle(const CookedAngle cooked)
{
   val = cooked.val - userOrigin.val;
}

RawAngle CookedAngle::hardwareOrigin = RawAngle(0);
bool CookedAngle::inverted = false;
CookedAngle UserAngle::userOrigin = CookedAngle(0);
