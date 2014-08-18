/*
 * mod.h
 *
 *  Created on: Aug 18, 2014
 *      Author: adrienp
 */

#ifndef MOD_H_
#define MOD_H_

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

// struct definition
#include "mav_struct.h"

// string manipulation

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

/** this is the id of the serial port that has been opened and that want to access **/
int fd;

// create object to hold all of the current uav status data
MAVInfo uav;

// command related global variables that are handy
int lastCmd = 0;


// just putting this here as a placeholder for now
void generateCommand();


#endif /* MOD_H_ */
