/*
 * common_vars.h
 *
 *  Created on: Aug 18, 2014
 *      Author: adrienp
 */

#ifndef COMMON_VARS_H_
#define COMMON_VARS_H_

#include "mav_struct.h"

// UNIQUE SYSTEM IDS, NEED TO BE DIFFERENT THAN ALL OTHER IDS TALKING TO THE PIXHAWK
int sysid = 16;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 101;

// Settings
int serial_compid = 0;


bool silent = true;              ///< Whether console output should be enabled

// for debugging (original debug settings)
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output

// for debugging purposes, whether or not you want stuff to be outputed to the command line
bool printAll = false;

// for when it is time to exit, stops the read thread.... NEED TO FIND A BETTER WAY TO STOP THE READ THREAD
bool closeAll = false;


// create object to hold all of the current uav status data
MAVInfo uav;

// command related global variables that are handy
int lastCmd = 0;

/** this is the id of the serial port that has been opened and that want to access **/
int fd;

/* whether or not we have received a heartbeat from the pixhawk */
bool heartbeatReceived = false;

/* current status of the tracking system
 * enumerated in TRACK_COMPUTER_STATUS
 * */
int status = 0;



#endif /* COMMON_VARS_H_ */
