/**
 * @file commander.cpp
 *
 * Definition of the set of functions handling sending commands to the pixhawk.
 * 
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <vector>
#include <sys/time.h>


// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#
#ifdef __linux
#include <sys/ioctl.h>
#endif

#include "libs/serial/mavlink_serial.h"
#include "common.h"
#include "commander.h"

// include what is needed ofr  the POMDP commands
#include "planner/pomdp.h"

using std::string;
using std::vector;
using namespace std;

/** rotation handling for DF antenna */
bool second_rotation_required = true;


void send_next_command(uint8_t &prev_state, uint8_t &new_state) {

	LOG_STATUS("[COMMANDER] the previous state was: %i\n", prev_state);

	vector<float> commands;
	float d_north = 0.0;
	float d_east = 0.0;
	float d_yaw = 0.0;
	float alt = common::flight_alt;
	switch (prev_state) {
		case TRACKING_HUNT_STATE_OFF:
		case TRACKING_HUNT_STATE_START:
		case TRACKING_HUNT_STATE_ROTATE:
			// rotating = false;	// NO LONGER NEEDED

			// if running emily config, will need to rotate twice
			if (common::emily && second_rotation_required) {
				send_df_mode(1);
				common::pixhawk->send_rotate_command(-1.0);
				second_rotation_required = false;
				break;
			}

			// get next command from the current planner
			commands = common::planner->action();
			LOG_STATUS("planner command received with length %d", commands.size());
			d_north = commands[0];
			d_east = commands[1];
			d_yaw = commands[2];

			// get the altitude, but not always set
			if (commands.size() > 3 && commands[3] > 0.0) {
				alt = commands[3];	
			}

			LOG_DEBUG("[COMMANDER] following tracking command (%f, %f)", d_north, d_east);
			common::pixhawk->send_tracking_command(d_north, d_east, alt);

			if (d_north == 1000.0) {
				LOG_STATUS("[COMMANDER] sending finish command\n");
				common::pixhawk->send_finish_command();
				return;
			}
			break;

		case TRACKING_HUNT_STATE_MOVE:
			// moving = false;		// NO LONGER NEEDED

			if (common::emily) {
				// make sure in "normal" mode of operation
				send_df_mode(0);
			}
		
			// send a rotate command
			common::pixhawk->send_rotate_command(-1.0);

			// management for emily config
			second_rotation_required = true;
			break;
		
	}
	
	return;
}


void send_df_mode(int mode) {

	if (mode == 1) {
		write(common::df_arduino->fd, "1", 1);
	} else {
		write(common::df_arduino->fd, "0", 1);
	}
}
