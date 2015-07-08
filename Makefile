CC=g++
CFLAGS=-c -Wall -std=c++11
INC=-I ~/git-uav/maker-of-decisions/mavlink
THREAD=-pthread

SOURCES = mod.cpp read_thread.cpp commander.cpp wifly_thread.cpp \
bearing_cc.cpp bearing_helper.cpp bearing_mle.cpp dirk_thread.cpp serial_lib/serial_port_class.cpp \
serial_lib/wifly_serial_class.cpp serial_lib/mavlink_serial_class.cpp

OBJECTS = $(SOURCES:.cpp=.o)

EXEC = mod

all: $(EXEC) cleano


$(EXEC): $(OBJECTS)
	$(CC) -Wall -std=c++11 $(INC) $^ -o $@ $(THREAD)

%.o: %.cpp
	$(CC) $(CFLAGS) $(INC) $< -o $@

serial_lib/%.o: serial_lib/%.cpp
	$(CC) $(CFLAGS) $(INC) $< -o $@

clean:
	\rm mod

cleano:
	\rm *.o serial_lib/*.o