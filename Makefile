CXXFLAGS = -std=c++11 -Wall

OBJECTS = mcontrol.o interface.o simulated.o controller.o angles.o defaults.o

#CPPFLAGS += -DHARDWARE -I/home/andrej/src/wiringPi/wiringPi
#CXXFLAGS += -L/home/andrej/src/wiringPi/wiringPi -lwiringPi
#OBJECTS += hardware.o

mcontrol: $(OBJECTS)
	g++ $(CXXFLAGS) -o mcontrol $(OBJECTS)

.PHONY: clean
clean:
	rm -f *.o mcontrol
