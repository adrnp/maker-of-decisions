// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <vector>

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

#include "common.h"
#include "system_ids.h"
#include "commander.h"

using std::string;
using std::vector;
using namespace std;

vector<float> cmd_north;
vector<float> cmd_east;
vector<float> cmd_alt;

int num_cmds = 0;
int cmd_index = 0;

bool load_move_commands() {

	ifstream cmd_file ("commands.csv");

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


void send_next_command(uint8_t &prev_state, uint8_t &new_state) {
	
	switch (prev_state) {
		case TRACKING_HUNT_STATE_OFF:
		case TRACKING_HUNT_STATE_START:
		case TRACKING_HUNT_STATE_ROTATE:
			rotating = false;

			/* send next command depending on flight mode */
			if (execute_tracking) {
				printf("should not be seeing this BAD!!!!\n");
				/*
				vector<float> commands = get_tracking_command();
				sendTrackingCommand(commands[0], commands[1]);
				*/
			} else {	
				// send the next move command
				sendMoveCommand();
			}
			break;
		case TRACKING_HUNT_STATE_MOVE:
			moving = false;
		
			// send a rotate command
			sendRotateCommand(-1.0);
			break;
		
	}
	
	return;
}

void sendTrackingCommand(float &north, float &east) {
	
	// retrieve the id of the last finished cmd
	int nextCmd = uav.last_cmd_finished_id++;

	// cycle the cmds (ids should go from 0 -> 3)
	if (cmd_index >= num_cmds) {
		cmd_index = 0;
	}

	// extract the next north and east commands
	float nextNorth = north;
	float nextEast = east;
	float nextAlt = flight_alt;

	printf("sending command %i: N %f\tE %f\tA %f\n", cmd_index, nextNorth, nextEast, nextAlt);

	cmd_index++;


	mavlink_tracking_cmd_t tracking_cmd;
	tracking_cmd.timestamp_usec = 0;
	tracking_cmd.north = nextNorth;
	tracking_cmd.east = nextEast;
	tracking_cmd.yaw_angle = 0.0;
	tracking_cmd.altitude = nextAlt;
	tracking_cmd.cmd_id = nextCmd;
	tracking_cmd.cmd_type = TRACKING_CMD_TRAVEL;

	mavlink_message_t message;
	mavlink_msg_tracking_cmd_encode(sysid, compid, &message, &tracking_cmd);

	/* printing stuff for debug purposes */
	printf("sending next move command\n");
	// int len = write_to_serial(message);
	// printf("Sent buffer of length %i\n",len);

	return;
}


void sendMoveCommand() {

	// retrieve the id of the last finished cmd
	int nextCmd = uav.last_cmd_finished_id++;

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


	mavlink_tracking_cmd_t tracking_cmd;
	tracking_cmd.timestamp_usec = 0;
	tracking_cmd.north = nextNorth;
	tracking_cmd.east = nextEast;
	tracking_cmd.yaw_angle = 0.0;
	tracking_cmd.altitude = nextAlt;
	tracking_cmd.cmd_id = nextCmd;
	tracking_cmd.cmd_type = TRACKING_CMD_TRAVEL;

	mavlink_message_t message;
	mavlink_msg_tracking_cmd_encode(sysid, compid, &message, &tracking_cmd);

	int len = pixhawk->write_serial(message);
	printf("sending next move command\n");
	// printf("Sent buffer of length %i\n",len);

	return;

}

void sendRotateCommand(float direction) {
	// retrieve the id of the last finished cmd
	int nextCmd = uav.last_cmd_finished_id++;


	mavlink_tracking_cmd_t tracking_cmd;
	tracking_cmd.timestamp_usec = 0;
	tracking_cmd.north = 0.0;			// don't travel any distance north
	tracking_cmd.east = 0.0;			// don't travel any distacne east
	tracking_cmd.yaw_angle = direction;		// rotate clockwise (NOTE: yaw angle no longer means yaw angle, but rather rotation direction)
	tracking_cmd.altitude = 0.0;		// will have pixhawk just use current altitude
	tracking_cmd.cmd_id = nextCmd;
	tracking_cmd.cmd_type = TRACKING_CMD_ROTATE;

	mavlink_message_t message;
	mavlink_msg_tracking_cmd_encode(sysid, compid, &message, &tracking_cmd);

	int len = pixhawk->write_serial(message);
	printf("sending next rotate command\n");
	// printf("Sent buffer of length %i\n",len);

	return;
}

void send_finish_command() {

	// retrieve the id of the last finished cmd
	int nextCmd = uav.last_cmd_finished_id++;

	mavlink_tracking_cmd_t tracking_cmd;

	tracking_cmd.timestamp_usec = 0;
	tracking_cmd.north = 0.0;			
	tracking_cmd.east = 0.0;			
	tracking_cmd.yaw_angle = 0.0;		
	tracking_cmd.altitude = 0.0;		
	tracking_cmd.cmd_id = nextCmd;
	tracking_cmd.cmd_type = TRACKING_CMD_FINISH;

	mavlink_message_t message;
	mavlink_msg_tracking_cmd_encode(sysid, compid, &message, &tracking_cmd);

	int len = pixhawk->write_serial(message);
	printf("sending finish command\n");
	// printf("Sent buffer of length %i\n",len);

	return;
}


void send_bearing_message(double &bearing, int32_t &lat, int32_t &lon, float &alt) {

	mavlink_bearing_cc_t bear;
	bear.bearing = bearing;
	bear.lat = lat;
	bear.lon = lon;
	bear.alt = alt;

	mavlink_message_t message;
	mavlink_msg_bearing_cc_encode(sysid, compid, &message, &bear);

	int len = pixhawk->write_serial(message);
	printf("sending bearing message\n");
	// printf("Sent buffer of length %i\n",len);

	return;
}


void send_rssi_message(int &rssi, int16_t &heading, int32_t &lat, int32_t &lon, float &alt) {
	mavlink_rssi_t rssi_msg;
	rssi_msg.rssi_value = rssi;
	rssi_msg.rssi_value2 = 0;
	rssi_msg.heading = (float) heading;
	rssi_msg.lat = 0; //lat;
	rssi_msg.lon = 0; //lon;
	rssi_msg.alt = 0.0; // alt;

	mavlink_message_t message;
	mavlink_msg_rssi_encode(sysid, compid, &message, &rssi_msg);

	int len = pixhawk->write_serial(message);
	printf("sending rssi message\n");
	// printf("Sent buffer of length %i\n", len);
	return;
}
