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

/** the file descriptor for the serial port connection */
// currently here, but may move to have all serial operations 
// be in the same class.
// hesitating on that, as don't know how that will work out with 
// potentially have multiple threads reading and writing....
extern int fd;

/** boolean stating whether or not we are currently in a rotating mode */
extern bool rotating;

/** boolean stating whether or not we are currently in a moving mode */
extern bool moving;



#endif /* COMMON_H_ */
