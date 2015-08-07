#!/bin/bash

# define the necessary variables

# pixhawk information
PIXHAWK_PORT=/dev/ttyUSB1
PIXHAWK_BAUD=115200

# wifly information
WIFLY1_PORT=/dev/ttyUSB0

# df antenna useage
DF_ANTENNA_ENABLED=0

if [ $DF_ANTENNA_ENABLED -eq 0 ]; then
	./mod -d $PIXHAWK_PORT -b $PIXHAWK_BAUD -w1 $WIFLY1_PORT -t -v
else
	./mod -d $PIXHAWK_PORT -b $PIXHAWK_BAUD -w1 $WIFLY1_PORT -t -v -e
fi
