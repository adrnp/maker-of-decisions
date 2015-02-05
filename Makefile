all: mod

mod: mod.o port_setup.o read_thread.o
	g++ mod.o port_setup.o serial_port.o -o mod -pthread

mod.o: mod.h mod.cpp port_setup.h read_thread.h mav_struct.h common_vars.h common_include.h
	g++ -I mavlink -Wall -c mod.cpp

port_setup.o: port_setup.cpp port_setup.h
	g++ -c -Wall port_setup.cpp

serial_port.o: serial_port.cpp serial_port.h
	g++ -c -Wall serial_port.cpp

read_thread.o: read_thread.cpp read_thread.h common_vars.h common_include.h
	g++ -c -I mavlink -Wall read_thread.cpp

clean:
	\rm *.o mod