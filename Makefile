all: mod

mod: mod.o serial_port.o read_thread.o commander.o serialwifly.o wifly_thread.o
	g++ mod.o serial_port.o read_thread.o commander.o serialwifly.o wifly_thread.o -o mod -pthread

mod.o: mod.h mod.cpp serial_port.h read_thread.h serialwifly.h wifly_thread.h mav_struct.h common.h
	g++ -I mavlink -Wall -c mod.cpp -pthread

serial_port.o: serial_port.cpp serial_port.h
	g++ -c -I mavlink -Wall serial_port.cpp

commander.o: commander.cpp commander.h
	g++ -c -I mavlink -Wall commander.cpp

read_thread.o: read_thread.cpp read_thread.h
	g++ -c -I mavlink -Wall read_thread.cpp

serialwifly.o: serialwifly.cpp serialwifly.h
	g++ -c -Wall -I mavlink serialwifly.cpp

wifly_thread.o: wifly_thread.cpp wifly_thread.h
	g++ -c -Wall -I mavlink wifly_thread.cpp


clean:
	\rm *.o mod