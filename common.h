/**
 * @file common.h
 *
 * The definition of a set of common variables used among the files.
 * Note the namespace (common) to designate these as common variables.
 *
 * @Author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <cstdlib>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>


#include "mav_struct.h"
#include "libs/serial/mavlink_serial.h"
#include "libs/planners/planner.h"

// some constants to determine tracking method
#define TRACK_PATH		0
#define TRACK_NAIVE 	1
#define TRACK_VARIABLE 	2
#define TRACK_POMDP 	3
#define TRACK_CIRCLE	4
#define TRACK_GREEDY	5
#define TRACK_MOMDP		6


namespace common {

	/** boolean to determine whether or not to show all outputs */
	extern bool verbose;

	/** boolean to determine whether or not to show debug outputs */
	extern bool debug;

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

	/** stuff needed for the tracking command */
	extern float flight_alt;
	extern int tracker_type;

	/** stuff needed for emily antenna */
	extern bool emily;

	/** the directory for all the log files for this run */
	extern std::string logfile_dir;

	/** the planner being used */
	extern Planner *planner;

	/** the logfile for all the main outputs */
	extern FILE *output_logfile;
}


inline void LOG_STATUS(std::string msg, ...) {
	// add a newline
	msg += "\n";

	// get the rest of the arguments
	va_list args;
	va_start(args, msg);

	// output
	vprintf(msg.c_str(), args);								// to command line
	msg.insert(0, "[STATUS]");
	vfprintf(common::output_logfile, msg.c_str(), args);	// to logfile

	// close the end of the arguments
	va_end(args);
}


inline void LOG_DEBUG(std::string msg, ...) {
	// add a newline
	msg += "\n";

	// get the rest of the arguments
	va_list args;
	va_start(args, msg);

	// output
	if (common::verbose) vprintf(msg.c_str(), args);		// to command line
	msg.insert(0, "[DEBUG]");
	vfprintf(common::output_logfile, msg.c_str(), args);	// to logfile

	// close the end of the arguments
	va_end(args);
}


inline void LOG_ERROR(std::string msg, ...) {
	// add a newline
	msg += "\n";
	msg.insert(0, "[ERROR]");

	// get the rest of the arguments
	va_list args;
	va_start(args, msg);

	// output
	vprintf(msg.c_str(), args);								// to command line
	vfprintf(common::output_logfile, msg.c_str(), args);	// to logfile

	// close the end of the arguments
	va_end(args);
}



#endif /* COMMON_H_ */
