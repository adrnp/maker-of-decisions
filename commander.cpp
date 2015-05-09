#include "common.h"
#include "serial_port.h"
#include "system_ids.h"
#include "commander.h"


float north[4] = {-30.0, 0.0, 30.0, 0.0};
float east[4] =  {0.0, 30.0, 0.0, -30.0};


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

	int len = write_to_serial(message);
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

	int len = write_to_serial(message);
	printf("sending finish command\n");
	// printf("Sent buffer of length %i\n",len);

	return;
}


void send_bearing_message(double &bearing, int32_t &lat, int32_t &lon, float &alt) {

	mavlink_bearing_t bear;
	bear.bearing = bearing;
	bear.lat = lat;
	bear.lon = lon;
	bear.alt = alt;

	mavlink_message_t message;
	mavlink_msg_bearing_encode(sysid, compid, &message, &bear);

	int len = write_to_serial(message);
	printf("sending bearing message\n");
	// printf("Sent buffer of length %i\n",len);

	return;
}


void send_rssi_message(int &rssi, int16_t &heading, int32_t &lat, int32_t &lon, float &alt) {
	mavlink_rssi_t rssi_msg;
	rssi_msg.rssi_value = rssi;
	rssi_msg.heading = (float) heading;
	rssi_msg.lat = 0; //lat;
	rssi_msg.lon = 0; //lon;
	rssi_msg.alt = 0.0; // alt;

	mavlink_message_t message;
	mavlink_msg_rssi_encode(sysid, compid, &message, &rssi_msg);

	int len = write_to_serial(message);
	printf("sending rssi message\n");
	// printf("Sent buffer of length %i\n", len);
	return;
}


int write_to_serial(mavlink_message_t &message) {
	
	// buffer needed for mavlink msg and function
	char buf[300];

	// Send message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write packet via serial link
	write(fd, buf, len);

	// Wait until all data has been written
	tcdrain(fd);

	return len;


}
