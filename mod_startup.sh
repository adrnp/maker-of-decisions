#!/bin/bash

# define the necessary variables

# pixhawk information
PIXHAWK_PORT=/dev/ttyUSB1
PIXHAWK_BAUD=115200

# wifly information
WIFLY1_PORT=/dev/ttyUSB0

WIFLY2_ENABLED=0
WIFLY2_PORT=/dev/ttyUSB2

if [ $WIFLY2_ENABLED -eq 0 ]; then
	./mod -d $PIXHAWK_PORT -b $PIXHAWK_BAUD -w1 $WIFLY1_PORT -c -v >> logFile.log
else
	./mod -d $PIXHAWK_PORT -b $PIXHAWK_BAUD -w1 $WIFLY1_PORT -w2 $WIFLY2_PORT -c -v
fi
