CXXFLAGS = -std=c++11 -Wall -lconfig++

OBJECTS = interface.o simulated.o controller.o angles.o mcontrol.o

ifdef HARDWARE
	OBJECTS += hardware.o

	CPPFLAGS += -I/home/andrej/src/wiringPi/wiringPi
	CXXFLAGS += -L/home/andrej/src/wiringPi/wiringPi

	CPPFLAGS += -DHARDWARE -DCONFIG_FILE_PATH=\"/etc\"
	CXXFLAGS += -lwiringPi
else
	CPPFLAGS += -DCONFIG_FILE_PATH=\".\"
endif

mcontrol: $(OBJECTS)
	g++ $(CXXFLAGS) -o mcontrol $(OBJECTS)

.PHONY: hardware
hardware:
	$(MAKE) HARDWARE=1

.PHONY: clean
clean:
	rm -f *.o mcontrol
