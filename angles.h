#ifndef ANGLES_H
#define ANGLES_H

class RawAngle;
class CookedAngle;
class UserAngle;

class RawAngle
{
public:
   RawAngle(float value) : val(value) {}
   float val;
};


class CookedAngle
{
public:
   CookedAngle(float value) : val(value) {}
   explicit CookedAngle(const RawAngle raw);
   explicit CookedAngle(const UserAngle user);
   float val;
   
   inline bool operator>(const CookedAngle& other)
      { return val > other.val; }

   inline bool operator>=(const CookedAngle& other)
      { return val >= other.val; }

   inline bool operator<=(const CookedAngle& other)
      { return val <= other.val; }

   static RawAngle hardwareOrigin;
   static bool inverted;
};

inline float operator-(const CookedAngle& a1, const CookedAngle& a2)
   { return a2.val - a1.val; }


class UserAngle
{
public:
   UserAngle(float value) : val(value) {}
   explicit UserAngle(const CookedAngle cooked);
   float val;

   static CookedAngle userOrigin;

   friend class CookedAngle;
};

float mod360(float value);

#endif // ANGLES_H
