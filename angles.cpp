#include <cmath>
#include "angles.h"

float mod360(float value)
{
   return value - 360.0 * std::floor(value / 360.0);
}

/* Explanation: raw, cooked and user angles:
 *
 * Raw angles are readouts that come directly from the sensor. These lay in
 * the range [0:360]
 *
 * Cooked angles are linearized and possibly inverted raw angles with the
 * origin positioned so that values around zero and 360 never occur (we are
 * relying on hardware end switches to prevent that). Still in the range
 * [0:360].
 *
 * User angles are cooked angles shifted by a user-defined value (to
 * reposition the origin). Values lie in the range [-origin:360-origin].
*/

CookedAngle::CookedAngle(const RawAngle raw)
{
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
