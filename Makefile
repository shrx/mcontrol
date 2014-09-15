CXXFLAGS = -std=c++11 -Wall

OBJECTS = mcontrol.o interface.o simulated.o
mcontrol: $(OBJECTS)
	g++ $(CXXFLAGS) -o mcontrol $(OBJECTS)

.PHONY: clean
clean:
	rm -f *.o mcontrol
