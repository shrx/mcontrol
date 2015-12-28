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
   ReturnValue retval = ReturnValue::Success;
   try {
      TCLAP::CmdLine cmd("Motor control");

      // Specification of command line parameters.
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

      // Parse the command line arguments.
      cmd.parse(argc, argv);

      ControllerParams cparams;

      // Initialize the controller parameters from the configuration file.
      try
      {
         cparams = ControllerParams(configFilename);
         if (arg_percentOutput.isSet() || !isatty(fileno(stdout)))
            cparams.indicatorStyle = ControllerParams::IndicatorStyle::Percent;
      }
      catch (libconfig::FileIOException& e)
      {
         std::cerr << "config file: could not read '" << configFilename << "'\n";
         throw ReturnValue::ConfigError;
      }
      catch (libconfig::ParseException& e)
      {
         std::cerr << "config file: error parsing '" << e.getFile()
                  << "', line " << e.getLine()
                  << ": " << e.getError() << std::endl;
         throw ReturnValue::ConfigError;
      }
      catch (libconfig::SettingTypeException& e)
      {
         std::cerr << "config file: wrong argument type for setting '" << e.getPath() << "'\n";
         throw ReturnValue::ConfigError;
      }
      catch (libconfig::SettingNotFoundException& e)
      {
         std::cerr << "config file: could not find setting '" << e.getPath() << "'\n";
         throw ReturnValue::ConfigError;
      }
      catch (ConfigFileException& e)
      {
         std::cerr << "config file: " << e.message << "\n";
         throw ReturnValue::ConfigError;
      }

      // Establish a controller with the parameters obtained above.
      Controller controller(cparams);

      if (arg_targetAngle.isSet())
      {
         // A slew is requested. Test whether the angle is within the safe limits
         // and perform the slew if everything seems OK.
         UserAngle targetAngle(arg_targetAngle.getValue());
         if (!targetAngle.isSafe())
         {
            std::cerr << "Target angle " << targetAngle.val
                        << " is not within safe limits ("
                        << UserAngle(CookedAngle::getMinimum()).val
                        << " <= target angle <= "
                        << UserAngle(CookedAngle::getMaximum()).val
                        << ").\nNot performing the slew.\n";
            throw(ReturnValue::ConfigError);
         }
         retval = controller.slew(CookedAngle(targetAngle));
      }
      else if (arg_park.isSet())
      {
         // A slew to the park position is requested. No need to test the safety
         // of the park position as this was already done when reading the config
         // file.
         retval = controller.slew(cparams.parkPosition);
      }
      else
      {
         // Only report the current axis angle (either raw angle or user angle,
         // depending on the command line switches).
         degrees angle;
         if (arg_queryRawAngle.isSet())
            angle = controller.getRawAngle().val;
         else
            angle = controller.getUserAngle().val;

         std::cout << angle << std::endl;
      }
   }
   catch (ReturnValue rv)
   {
      retval = rv;
   }

   return static_cast<int>(retval);
}
