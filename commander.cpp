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
#include "tracker.h"

// include what is needed ofr  the POMDP commands
#include "planner/pomdp.h"

using std::string;
using std::vector;
using namespace std;

/* needed for emily antenna logic */
bool second_rotation_required = true;

vector<float> cmd_north;
vector<float> cmd_east;
vector<float> cmd_alt;

int num_cmds = 0;
int cmd_index = 0;

bool load_move_commands() {
	string file_name(common::command_file);
	ifstream cmd_file (file_name);

	if (!cmd_file.is_open()) {
		// means there is an error in loading the file
		return false;
	}

	// make sure vectors are clean
	cmd_north.clear();
	cmd_east.clear();
	cmd_alt.clear();

	float cmdN;
	float cmdE;
	float cmdA;
	char comma;
	while (cmd_file >> cmdN >> comma >> cmdE >> comma >> cmdA) {
		cmd_north.push_back(cmdN);
		cmd_east.push_back(cmdE);
		cmd_alt.push_back(cmdA);

		printf("North command: %f\n", cmdN);
		printf("East cmd: %f\n", cmdE);
		printf("Alt cmd: %f\n", cmdA);
	}

	printf("num commands read: %d\n", num_cmds);

	num_cmds = cmd_north.size();
	return true;
}


void send_next_command(uint8_t &prev_state, uint8_t &new_state, double &bearing, int &rssi) {

	printf("[COMMANDER] the previous state was: %i\n", prev_state);

	vector<float> commands;
	std::pair<float, float> commands_pair;	
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

			/* send next command depending on flight mode */
			if (common::execute_tracking) {
				printf("[COMMANDER] sending a tracking command\n");
				// get the next command, which depends on the tracking method desired
				float d_north = 0.0;
				float d_east = 0.0;
				switch (common::tracker_type) {
					case TRACK_NAIVE:
						if (common::verbose) printf("[COMMANDER] naive tracking command being made...\n");
						commands = calc_next_command(bearing, rssi);
						d_north = commands[0];
						d_east = commands[1];
						break;
					case TRACK_VARIABLE:
						if (common::verbose) printf("[COMMANDER] variable tracking command being made...\n");
						commands = calc_next_command_variable(bearing, rssi);
						d_north = commands[0];
						d_east = commands[1];
						break;
					case TRACK_POMDP:
						if (common::verbose) printf("[COMMANDER] pomdp tracking command being made...\n");
						commands_pair = get_next_pomdp_action(bearing, rssi);
						d_north = commands_pair.first;
						d_east = commands_pair.second;

						if (d_north == 1000.0) {
							printf("[COMMANDER] sending finish command\n");
							common::pixhawk->send_finish_command();
							return;
						}

						break;
				}
				
				if (common::verbose) printf("[COMMANDER] following tracking command (%f, %f)\n", d_north, d_east);
				common::pixhawk->send_tracking_command(d_north, d_east, 360.0);
				
				
			} else {
				if (common::verbose) printf("[COMMANDER] sending the next preset move command\n");	
				// send the next move command
				send_move_command();
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



void send_move_command() {

	// cycle the cmds (ids should go from 0 -> 3)
	if (cmd_index >= num_cmds) {
		cmd_index = 0;
	}

	printf("[COMMANDER] sending move command with index: %d\n", cmd_index);

	// extract the next north and east commands
	float nextNorth = cmd_north[cmd_index];
	float nextEast = cmd_east[cmd_index];
	float nextAlt = cmd_alt[cmd_index];

	printf("[COMMANDER] sending command %i: N %f\tE %f\tA %f\n", cmd_index, nextNorth, nextEast, nextAlt);

	cmd_index++;

	common::pixhawk->send_tracking_command(nextNorth, nextEast, nextAlt);

	return;

}


void send_df_mode(int mode) {

	if (mode == 1) {
		write(common::df_arduino->fd, "1", 1);
	} else {
		write(common::df_arduino->fd, "0", 1);
	}
}
