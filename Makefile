all: mod

mod: mod.o serial_port.o read_thread.o commander.o serialwifly.o wifly_thread.o bearing_cc.o bearing_helper.o bearing_mle.o dirk_thread.o
	g++ mod.o serial_port.o read_thread.o commander.o serialwifly.o wifly_thread.o bearing_cc.o bearing_helper.o bearing_mle.o dirk_thread.o -o mod -pthread

mod.o: mod.h mod.cpp serial_port.h read_thread.h serialwifly.h wifly_thread.h mav_struct.h common.h bearing.h dirk_thread.h
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

bearing_cc.o: bearing_cc.cpp bearing.h
	g++ -std=c++11 -c -Wall bearing_cc.cpp

bearing_helper.o: bearing_helper.cpp bearing.h
	g++ -std=c++11 -c -Wall bearing_helper.cpp

bearing_mle.o: bearing_mle.cpp bearing.h
	g++ -std=c++11 -c -Wall bearing_mle.cpp

dirk_thread.o: dirk_thread.cpp dirk_thread.h
	g++ -c -Wall -I mavlink dirk_thread.cpp

clean:
	\rm *.o mod
