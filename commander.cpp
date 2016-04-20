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

#include "serial_lib/mavlink_serial.h"
#include "common.h"
#include "commander.h"
#include "tracker.h"

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
	string file_name(command_file);
	ifstream cmd_file ("commands/" + file_name + ".csv");

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

	 printf("the previous state was: %i\n", prev_state);
	
	switch (prev_state) {
		case TRACKING_HUNT_STATE_OFF:
		case TRACKING_HUNT_STATE_START:
		case TRACKING_HUNT_STATE_ROTATE:
			// rotating = false;	// NO LONGER NEEDED

			// if running emily config, will need to rotate twice
			if (emily && second_rotation_required) {
				send_df_mode(1);
				pixhawk->send_rotate_command(-1.0);
				second_rotation_required = false;
				break;
			}

			/* send next command depending on flight mode */
			if (execute_tracking) {
				 printf("sending a tracking command\n");

				vector<float> commands = calc_next_command_variable(bearing, rssi);
				
				if (verbose) printf("following tracking command (%f, %f)\n", commands[0], commands[1]);
				float commandNorth = commands[0];
				float commandEast = commands[1];
				pixhawk->send_tracking_command(commandNorth, commandEast, 360.0);
				
			} else {
				if (verbose) printf("sending the next preset move command\n");	
				// send the next move command
				send_move_command();
			}
			break;
		case TRACKING_HUNT_STATE_MOVE:
			// moving = false;		// NO LONGER NEEDED

			if (emily) {
				// make sure in "normal" mode of operation
				send_df_mode(0);
			}
		
			// send a rotate command
			pixhawk->send_rotate_command(-1.0);

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
	printf("sending move command with index: %d\n", cmd_index);

	// extract the next north and east commands
	float nextNorth = cmd_north[cmd_index];
	float nextEast = cmd_east[cmd_index];
	float nextAlt = cmd_alt[cmd_index];
	printf("sending command %i: N %f\tE %f\tA %f\n", cmd_index, nextNorth, nextEast, nextAlt);

	cmd_index++;

	pixhawk->send_tracking_command(nextNorth, nextEast, nextAlt);

	return;

}


void send_df_mode(int mode) {

	if (mode == 1) {
		write(df_arduino->fd, "1", 1);
	} else {
		write(df_arduino->fd, "0", 1);
	}
}
