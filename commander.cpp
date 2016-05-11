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

/** allow non-rotations at end of move */
bool rotate_after_move = true;

void execute_action(const Action &action) {

	// first need to parse the action that was received
	// for now only parse a small subset of these actions...
	
	float d_north = 0.0;
	float d_east = 0.0;
	float yaw = 0.0;
	float alt = common::flight_alt;

	// get the validity flags and behavioral flags
	int valid = action.valid;
	int flags = action.flags;

	if ((valid & Action::VALID_NORTH) > 0) {
		d_north = action.north;
	}

	if ((valid & Action::VALID_EAST) > 0) {
		d_east = action.east;
	}

	if ((valid & Action::VALID_ALTITUDE) > 0) {
		alt = action.altitude;
	}

	if ((valid & Action::VALID_YAW) > 0) {
		// now need to determine if yaw is relative or absolute
		if ((flags & Action::FLAG_YAW_ABSOLUTE) > 0) {
			yaw = action.yaw;
		} else {
			yaw = common::uav.vfr_hud.heading + action.yaw;
			if (yaw >= 360) {
				yaw -= 360;
			} else if (yaw < 0) {
				yaw += 360;
			}
		}
	}

	if ((valid & Action::VALID_ROTATION_ANGLE) > 0) {
		if (action.rotation_angle == 0.0) {

			// tell the vehicle not to rotate after moving, move again instead
			rotate_after_move = false;
		}
	}

	// TODO: implement the rest of the different possibilities
	

	// send the commands to the pixhawk
	// check to see if rotating again or moving
	if (d_north == 0 && d_east == 0) {
		common::pixhawk->send_rotate_command(-1.0);
	} else if (d_north >= 999.0 || d_north <= -999.0) {
		LOG_STATUS("[COMMANDER] sending finish command\n");
		common::pixhawk->send_finish_command();
	} else {
		common::pixhawk->send_tracking_command(d_north, d_east, yaw, alt);
	}

}




void send_next_command(uint8_t &prev_state, uint8_t &new_state) {

	LOG_STATUS("[COMMANDER] the previous state was: %i\n", prev_state);


	Action action;

	vector<float> commands;
	float d_north = 0.0;
	float d_east = 0.0;
	float yaw_angle = 270.0;
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

			// get the next action and execute it
			execute_action(common::planner->action());

			break;

		case TRACKING_HUNT_STATE_MOVE:
			// moving = false;		// NO LONGER NEEDED

			if (common::emily) {
				// make sure in "normal" mode of operation
				send_df_mode(0);
			}

			if (rotate_after_move) {
				// send a rotate command
				common::pixhawk->send_rotate_command(-1.0);
			} else {
				rotate_after_move = true;
				execute_action(common::planner->action());
			}

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
