#include <iostream>
#include <vector>
#include <tclap/CmdLine.h>
#include <cstdio>
#include <unistd.h>
#include <libconfig.h++>
#include "controller.h"

#ifndef CONFIG_FILE_PATH
#define CONFIG_FILE_PATH "."
#endif

const char* configFilename = CONFIG_FILE_PATH "/mcontrol.conf";

int main(int argc, char *argv[])
{
   TCLAP::CmdLine cmd("Motor control");

   TCLAP::SwitchArg arg_queryAngle("q", "query-angle", "Query angle");
   TCLAP::SwitchArg arg_queryRawAngle("r", "raw-angle", "Query raw angle");
   TCLAP::SwitchArg arg_park("", "park", "Slew to park position");
   TCLAP::UnlabeledValueArg<degrees> arg_targetAngle(
      "angle", "Slew to this angle", false, 0, "target angle");

   auto xorArgs = std::vector<TCLAP::Arg*>{
      &arg_queryAngle,
      &arg_queryRawAngle,
      &arg_park,
      &arg_targetAngle};

   cmd.xorAdd(xorArgs);

   TCLAP::SwitchArg arg_percentOutput("p", "percent",
      "Show slew progress by outputting lines with '<angle> <slew percent>' "
      "(default when standard output is not a tty)."
   );
   cmd.add(arg_percentOutput);

   cmd.parse(argc, argv);

   ControllerParams cparams;

   try
   {
      cparams = ControllerParams(configFilename);
      if (arg_percentOutput.isSet() || !isatty(fileno(stdout)))
         cparams.indicatorStyle = ControllerParams::IndicatorStyle::Percent;
   }
   catch (libconfig::FileIOException& e)
   {
      std::cerr << "config file: could not read '" << configFilename << "'\n";
      exit(1);
   }
   catch (libconfig::ParseException& e)
   {
      std::cerr << "config file: error parsing '" << e.getFile()
                << "', line " << e.getLine()
                << ": " << e.getError() << std::endl;
      exit(1);
   }
   catch (libconfig::SettingTypeException& e)
   {
      std::cerr << "config file: wrong argument type for setting '" << e.getPath() << "'\n";
      exit(1);
   }
   catch (libconfig::SettingNotFoundException& e)
   {
      std::cerr << "config file: could not find setting '" << e.getPath() << "'\n";
      exit(1);
   }
   catch (ConfigFileException& e)
   {
      std::cerr << "config file: " << e.message << "\n";
      exit(1);
   }

   Controller controller(cparams);

   if (arg_targetAngle.isSet())
   {
      UserAngle targetAngle(arg_targetAngle.getValue());
      if (!targetAngle.isSafe())
      {
         std::cerr << "Target angle " << targetAngle.val
                      << " is not within safe limits ("
                      << UserAngle(CookedAngle::getMinimum()).val
                      << " <= target angle <= "
                      << UserAngle(CookedAngle::getMaximum()).val
                      << ").\nNot performing the slew.\n";
         exit(1);
      }
      controller.slew(CookedAngle(targetAngle));
   }
   else if (arg_park.isSet())
   {
      controller.slew(cparams.parkPosition);
   }
   else
   {
      degrees angle;
      if (arg_queryRawAngle.isSet())
         angle = controller.getRawAngle().val;
      else
         angle = controller.getUserAngle().val;

      std::cout << angle << std::endl;
   }

   return 0;
}
