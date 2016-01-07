mcontrol - declination axis control for PAART[*]
================================================

[*] PAART: the amateur radio telescope of
    Astronomical Society Vega - Ljubljana


DESCRIPTION
-----------

This program has two modes of operation: a query mode that determines and
prints out the current axis orientation and a slew mode that rotates the
axis to the specified orientation (angle). It can perform a linearization of
the raw sensor readouts and adjust the angle scale according to the user's
needs. Various parameters, such as the linearization coefficients, angle
scale, motor speed, acceleration etc. can be adjusted in a configuration
file.


HARDWARE
--------

The program is intended to be run on a Raspberry Pi connected to a custom
circuit consisting of an AS5048A Magnetic Rotary Encoder and a brushed DC
motor. The rotary encoder senses the current orientation of the axis and
reports the readouts via SPI bus to the R-Pi. Two GPIO pins of the computer
are used to control a H-bridge composed of two relays (for setting the
direction of spinning) and a GPIO pin with PWM capability is connected to a
power MOSFET that controls the amount of power delivered to the motor and
hence the motor speed.

Due to the program being tailored to a specific circuit of a particular
telescope, it will almost surely require modifications in case anyone tries
to use in a different setting. However, the number of modifications is
expected to be few and the general framework should work well even for
different hardware. In case that anyone adapts this software for their use
or just finds it useful in any other way, we would be more than happy to
hear from them!

mcontrol was developed with an awareness that the actual hardware might not
be available at the computer where the development takes place and that even
if it was, mistakes made during the development might harm the hardware.
Therefore, it provides within the source code a simulator of the
motor+sensor assembly that can be used as virtual hardware for the purpose
of development (see INSTALLATION below on how to enable/disble the support
for real hardware).


SOFTWARE REQUIREMENTS
---------------------

mcontrol requires the following libraries:

* libconfig++ (http://www.hyperrealm.com/libconfig/)
* TCLAP (http://tclap.sourceforge.net)

For controlling the actual hardware (as opposed to the simulator), the
following library is additionally needed:

* WiringPi (http://wiringpi.com/)


INSTALLATION
------------

mcontrol is built using CMake. If you are not familiar with CMake, refer to
any tutorial. But for starters, run these commands from the mcontrol source
directory:

  mkdir build
  cd build
  cmake ..
  make

NOTE: by default, mcontrol is built with a simulator (virtual hardware). If
you want to control the actual hardware, you need to edit the CMakeCache.txt
file in the build directory and change the variable HARDWARE to ON
(i.e., HARDWARE:BOOL=ON).

The HARDWARE variable also changes the path where mcontrol looks for its
configuration file: with HARDWARE set to OFF, it expects to find
mcontrol.conf in the current directory, whereas with the HARDWARE set to ON,
it tries to open /etc/mcontrol.conf.

The compiled executable lies in the build directory and you can run it from
there or copy it to a directory within your $PATH. Note that with hardware
support enabled, mcontrol requires superuser privileges to run due to the
need for direct access to GPIO pins. To give access to ordinary users, the
executable can be installed with root as the owner and the SetUID bit
enabled.


OPERATION
---------

A summary of options can be obtained by running "mcontrol --help".

In query mode, mcontrol can report the current axis position in terms of two
types of angles: raw angles (unprocessed values obtained directly from the
sensor) and ordinary (user) angles: linearized, possibly inverted and origin
shifted values. In the majority of cases, only the latter are of interest to
the user. However, the configuration file requires some variables to be set
in terms of the raw angles (such as the park position) and the ability to
query them comes handy.

In slew mode, a single command line parameter, namely the target angle, is
given to mcontrol and the program performs the slew according to the
parameters (acceleration, maximum power etc.) specified in the configuration
file.

Before any slews are performed on new hardware, it is mandatory to review
the configuration file carefully and check if any of the parameters need
adjustment. Failure to do so can lead to mcontrol moving the axis past the
allowed limits, taking the wrong path to the target angle or just causing
confusion by establishing an arbitrary angle scale. The configuration
parameters are extensively documented in the configuration file itself (see
sample mcontrol.conf).

Even if you are completely sure about getting the settings right, design the
hardware so that it can, to the best of its ability, withstand software
malfunctions or operator errors (e.g., install end switches that disconnect
the motor at the end of the allowed range). Don't come yelling at us if your
device decides to make stretched noodles out of its own cables. :-)


LICENSING INFORMATION
---------------------

mcontrol is provided under the GNU General Public License, version 3 or
later. See COPYING for more information.

Copyright 2014 Andrej Lajovic <andrej.lajovic@ad-vega.si>
