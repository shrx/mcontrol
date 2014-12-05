/*
 *    mcontrol, declination axis control for PAART, the radiotelescope of
 *              Astronomical Society Vega - Ljubljana
 *
 *    Copyright (C) 2014 Andrej Lajovic <andrej.lajovic@ad-vega.si>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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


void CookedAngle::setSafeLimits(const CookedAngle min, const CookedAngle max)
{
   minimumSafeAngle = min;
   maximumSafeAngle = max;
}


bool CookedAngle::isSafe() const
{
   return (val >= minimumSafeAngle.val) && (val <= maximumSafeAngle.val);
}


UserAngle::UserAngle(const CookedAngle cooked)
{
   val = cooked.val - userOrigin.val;
}


void UserAngle::setOrigin(const CookedAngle origin)
{
   userOrigin = origin;
}


bool UserAngle::isSafe() const
{
   return CookedAngle(*this).isSafe();
}

// Definition of static class members.
std::vector<float> CookedAngle::linCoeffs;
RawAngle CookedAngle::hardwareOrigin = RawAngle(0);
degrees CookedAngle::offset = 0;
bool CookedAngle::inverted = false;
CookedAngle CookedAngle::minimumSafeAngle{0};
CookedAngle CookedAngle::maximumSafeAngle{360};
CookedAngle UserAngle::userOrigin = CookedAngle(0);
