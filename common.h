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

/** MAVInfo object containing the current state information of the uav */
extern MAVInfo uav;

/** flag to determine whether or not the read and write should be running */
extern int RUNNING_FLAG;




#endif /* COMMON_H_ */
