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
#include "libs/serial/mavlink_serial.h"

namespace common {

	/** boolean to determine whether or not to show all outputs */
	extern bool verbose;

	/** boolean to determine whether or not to show debug outputs */
	extern bool debug;

	/** use command file */
	extern bool get_commands;
	extern const char* command_file;

	/** MAVInfo object containing the current state information of the uav */
	extern MAVInfo uav;

	/** flag to determine whether or not the read and write should be running */
	extern int RUNNING_FLAG;

	/** the connection to the pixhawk */
	extern MavlinkSerial* pixhawk;

	/** the connection to the DF arduino */
	extern SerialPort* df_arduino;

	/** wifly device locations */
	extern const char *sensor_port;
	extern const char *omni_wifly_port;

	extern bool dual_wifly;

	/* stuff needed for the tracking command */
	extern bool execute_tracking;
	extern float flight_alt;
	extern int tracker_type;

	/* stuff needed for emily antenna */
	extern bool emily;

	/* the directory for all the log files for this run */
	extern std::string logfile_dir;
}



#endif /* COMMON_H_ */
