#ifndef ANGLES_H
#define ANGLES_H

/* Explanation: raw, cooked and user angles:
 *
 * Raw angles are readouts that come directly from the sensor. These lay in
 * the range [0:360]
 *
 * Cooked angles are linearized and possibly inverted raw angles with the
 * origin positioned so that values around zero and 360 never occur (we are
 * relying on hardware end switches to prevent that). Still in the range
 * [0:360]. This is the type the controller uses in its calculations.
 *
 * User angles are cooked angles shifted by a user-defined value (to
 * reposition the origin). Values lie in the range [-origin:360-origin].
 * User angles are meant to be used purely within the context of interaction
 * with the user and not for other internal calculations.
*/

class RawAngle;
class CookedAngle;
class UserAngle;


/* RawAngle is not a subclass of Angle (declared below) because it does not
 * represent a true angle due to the possible nonlinearities in the
 * measurement. No arithmetic, then: just values.
*/

class RawAngle
{
public:
   explicit RawAngle(float value) : val(value) {}
   float val;
};


/* A general angle class.
 *
 * We use a template to keep all operators in one place but prevent
 * operations between incompatible derived angle types. I.e. calculating
 * (UserAngle(float) - CookedAngle(float)) makes no sense and should
 * therefore throw a compile-time error.
*/

template <class DerivedAngle>
class Angle
{
public:
   Angle() = default;
   explicit Angle(float value) : val(value) {}

   // comparison operators
   inline bool operator>(const DerivedAngle& other) { return val > other.val; }
   inline bool operator<(const DerivedAngle& other) { return val < other.val; }
   inline bool operator>=(const DerivedAngle& other) { return val >= other.val; }
   inline bool operator<=(const DerivedAngle& other) { return val <= other.val; }

   // difference between two angles is an ordinary float value
   inline float operator-(const DerivedAngle& other) { return val - other.val; }

   float val;
};


class CookedAngle : public Angle<CookedAngle>
{
public:
   explicit CookedAngle(float value) : Angle(value) {};
   explicit CookedAngle(const RawAngle raw);
   explicit CookedAngle(const UserAngle user);

   static RawAngle hardwareOrigin;
   static bool inverted;
};


class UserAngle : public Angle<UserAngle>
{
public:
   explicit UserAngle(float value) : Angle(value) {};
   explicit UserAngle(const CookedAngle cooked);

   static CookedAngle userOrigin;

   // allow CookedAngle to access userOrigin
   friend class CookedAngle;
};

float mod360(float value);

#endif // ANGLES_H
