/*
 * common.h
 *
 * this file contains variables that will be common throughout all of the files.
 * 
 * Note: the actual declaration of these variables can be found in the mod.cpp file
 *
 *  Created on: Jan 23, 2015
 *      Author: adrienp
 */

#ifndef COMMON_H_
#define COMMON_H_

 #include "mav_struct.h"
 #include "serial_lib/mavlink_serial.h"


/** boolean to determine whether or not to show all outputs */
extern bool verbose;

/** boolean to determine whether or not to show debug outputs */
extern bool debug;

/** use command file */
extern bool get_commands;

/** MAVInfo object containing the current state information of the uav */
extern MAVInfo uav;

/** flag to determine whether or not the read and write should be running */
extern int RUNNING_FLAG;

/** the connection to the pixhawk */
extern MavlinkSerial* pixhawk;

/** the connection to the DF arduino */
extern SerialPort* df_arduino;

/** boolean stating whether or not we are currently in a rotating mode */
extern bool rotating;

/** boolean stating whether or not we are currently in a moving mode */
extern bool moving;

/** wifly device locations */
extern char * wifly_port1;
extern char * wifly_port2;

extern bool dual_wifly;

/* stuff needed for the tracking command */
extern bool execute_tracking;
extern float flight_alt;

/* stuff needed for emily antenna */
extern bool emily;

#endif /* COMMON_H_ */
