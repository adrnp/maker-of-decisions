CC=g++
CFLAGS=-c
CFLAGS2=-Wall -std=c++11
INC=-I ~/GitDocs/maker-of-decisions/mavlink
THREAD=-pthread

SOURCES = mod.cpp read_thread.cpp commander.cpp tracker.cpp wifly2_thread.cpp wifly_thread.cpp dirk_thread.cpp \
bearing_lib/bearing_cc.cpp bearing_lib/bearing_helper.cpp bearing_lib/bearing_mle.cpp \
bearing_lib/bearing_max.cpp bearing_lib/bearing_max3.cpp \
serial_lib/serial_port.cpp serial_lib/wifly_serial.cpp serial_lib/mavlink_serial.cpp \
planner/belief.cpp planner/pomdp.cpp planner/pomdp_math.cpp

OBJECTS = $(SOURCES:.cpp=.o)

EXEC = mod

all: $(EXEC) cleano


$(EXEC): $(OBJECTS)
	$(CC) $(CFLAGS2) $(INC) $^ -o $@ $(THREAD)

%.o:%.cpp
	$(CC) $(CFLAGS) $(CFLAGS2) $(INC) $< -o $@

serial_lib/%.o: serial_lib/%.cpp serial_lib/serial_port.h
	$(CC) $(CFLAGS) $(CFLAGS2) $(INC) $< -o $@

bearing_lib/%.o: bearing_lib/%.cpp bearing_lib/bearing.h
	$(CC) $(CFLAGS) $(CFLAGS2) $(INC) $< -o $@

planner/%.o: planner/%.cpp planner/pomdp.h
	$(CC) $(CFLAGS) $(CFLAGS2) $(INC) $< -o $@

clean:
	\rm mod

cleano:
	\rm *.o serial_lib/*.o bearing_lib/*.o planner/*.o
