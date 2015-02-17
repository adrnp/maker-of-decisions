all: mod

mod: mod.o serial_port.o read_thread.o commander.o
	g++ mod.o serial_port.o read_thread.o commander.o -o mod -pthread

mod.o: mod.h mod.cpp serial_port.h read_thread.h mav_struct.h common.h
	g++ -I mavlink -Wall -c mod.cpp -pthread

serial_port.o: serial_port.cpp serial_port.h
	g++ -c -I mavlink -Wall serial_port.cpp

commander.o: commander.cpp commander.h
	g++ -c -I mavlink -Wall commander.cpp

read_thread.o: read_thread.cpp read_thread.h
	g++ -c -I mavlink -Wall read_thread.cpp

clean:
	\rm *.o mod