#include "common.h"
#include "serial_port.h"
#include "system_ids.h"
#include "commander.h"


float north[4] = {-10.0, 0.0, 10.0, 0.0};
float east[4] =  {0.0, 10.0, 0.0, -10.0};


void sendNextCommand() {

	// retrieve the id of the last finished cmd
	int nextCmd = uav.last_cmd_finished_id++;

	// cycle the cmds (ids should go from 0 -> 3)
	if (nextCmd > 3) {
		nextCmd = 0;
	}

	// extract the next north and east commands
	float nextNorth = north[nextCmd];
	float nextEast = east[nextCmd];

	mavlink_tracking_cmd_t tracking_cmd;
	tracking_cmd.timestamp_usec = 0;
	tracking_cmd.north = nextNorth;
	tracking_cmd.east = nextEast;
	tracking_cmd.yaw_angle = 0.0;
	tracking_cmd.altitude = 60.0;
	tracking_cmd.cmd_id = nextCmd;
	tracking_cmd.cmd_type = TRACKING_CMD_TRAVEL;

	mavlink_message_t message;
	mavlink_msg_tracking_cmd_encode(sysid, compid, &message, &tracking_cmd);

	int len = write_to_serial(message);
	printf("Sent buffer of length %i\n",len);

	return;

}



int write_to_serial(mavlink_message_t &message) {
	
	// buffer needed for mavlink msg send function
	char buf[300];

	// Send message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write packet via serial link
	write(fd, buf, len);

	// Wait until all data has been written
	tcdrain(fd);

	return len;


}