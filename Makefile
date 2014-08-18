all: mod

mod: mod.o port_setup.o read_thread.o
	g++ mod.o port_setup.o -o mod -pthread

mod.o: mod.h mod.cpp port_setup.h read_thread.h mav_struct.h
	g++ -I mavlink -Wall -c mod.cpp

port_setup.o: port_setup.cpp port_setup.h
	g++ -c -Wall port_setup.cpp

read_thread.o: read_thread.cpp read_thread.h
	g++ -c -I mavlink -Wall read_thread.cpp

clean:
	\rm *.o mod