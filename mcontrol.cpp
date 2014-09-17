#include <iostream>
#include <vector>
#include <tclap/CmdLine.h>
#include "controller.h"

extern ControllerParams cparams;


class TCLAPangleConstraint : public TCLAP::Constraint<degrees>
{
public:
   std::string description() const { return "0 <= angle < 360"; }
   std::string shortID() const { return "angle"; }
   bool check(const degrees& value) const { return (value >= 0 && value < 360); }
} angleConstraint;


int main(int argc, char *argv[])
{
   TCLAP::CmdLine cmd("Motor control");
   
   TCLAP::SwitchArg arg_queryAngle("q", "query-angle", "Query angle");
   TCLAP::SwitchArg arg_queryCookedAngle("c", "cooked-angle", "Query cooked angle");
   TCLAP::SwitchArg arg_queryRawAngle("r", "raw-angle", "Query raw angle");
   TCLAP::UnlabeledValueArg<degrees> arg_targetAngle(
      "angle", "Slew to this angle", false, 0, &angleConstraint);
   
   auto xorArgs = std::vector<TCLAP::Arg*>{
      &arg_queryAngle,
      &arg_queryCookedAngle,
      &arg_queryRawAngle,
      &arg_targetAngle};

   cmd.xorAdd(xorArgs);
   cmd.parse(argc, argv);
   
   Controller controller(cparams);

   if (arg_targetAngle.isSet())
   {
      auto targetAngle = UserAngle(arg_targetAngle.getValue());
      controller.slew(CookedAngle(targetAngle));
   }
   else
   {
      degrees angle;
      if (arg_queryRawAngle.isSet())
         angle = controller.getRawAngle().val;
      else if (arg_queryCookedAngle.isSet())
         angle = controller.getCookedAngle().val;
      else
         angle = controller.getUserAngle().val;
      
      std::cout << angle << std::endl;
   }

   return 0;
}
