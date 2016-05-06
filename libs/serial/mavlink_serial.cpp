// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
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


#include "mavlink_serial.h"
#include "system_ids.h"

MavlinkSerial::MavlinkSerial(std::string logfile_dir) : 
SerialPort(logfile_dir),
_command_id(0)
{
	printf("default mavlink serial constructor\n");
}

MavlinkSerial::MavlinkSerial(std::string logfile_dir, bool verbose) : 
SerialPort(logfile_dir, verbose),
_command_id(0)
{
	// nothing specific to do in this constructor
	printf("verbose mavlink serial constructor\n");
}

MavlinkSerial::MavlinkSerial(std::string logfile_dir, bool verbose, const char* &uart_name,  const int &baudrate) : 
SerialPort(logfile_dir, verbose, uart_name, baudrate),
_command_id(0)
{
	printf("complex mavlink serial constructor\n");
	printf("fd = %i\n", fd);
}

MavlinkSerial::~MavlinkSerial() {
	// nothing to do in the destructor
	// TODO: should probably close the file descriptor...
}

int MavlinkSerial::get_fd() {
	printf("get fd = %i\n", fd);
	return fd;
}


uint8_t MavlinkSerial::read_serial(mavlink_status_t *lastStatus, mavlink_message_t *message) {
	// variables needed for the message reading
	uint8_t cp;					// not sure
	mavlink_status_t status;	// current message status
	uint8_t msgReceived = false; // whether or not a message was correctly received

	// read in from the file
	if (read(fd, &cp, 1) > 0) {

		// Check if a message could be decoded, return the message in case yes
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, message, &status);

		// check the packet drop count to see if there was a packet dropped during this message reading
		if (lastStatus->packet_rx_drop_count != status.packet_rx_drop_count) {

			// print out some error information containing dropped packet indo
			//LOG_DEBUG("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			
			// print out the characters of the packets themselves
			/*
			if (_verbose) {
				unsigned char v=cp;
				fprintf(stderr,"%02x ", v);
			}
			*/
		}

		// update the last message status 
		*lastStatus = status;

	} else { // means unable to read from the serial device

		// print out error as needed
		LOG_ERROR("ERROR: Could not read from fd %d\n", fd);
	}

	// return whether or not the message was received
	return msgReceived;
}


int MavlinkSerial::write_serial(mavlink_message_t &message) {

	LOG_DEBUG("writing to pixhawk...");

	// buffer needed for mavlink msg and function
	char buf[300];

	// Send message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write packet via serial link
	write(fd, buf, len);

	// Wait until all data has been written
	tcdrain(fd);

	LOG_DEBUG("wrote message of length %u", len);

	return len;


}



// the mavlink specific functions



void MavlinkSerial::send_tracking_command(const float &north, const float &east, const float &alt) {

	// TODO: add altitude as a tracking command input!!! (this is horrible to have it as a default here)
	
	// retrieve the id of the last finished cmd
	int nextCmd = _command_id;
	_command_id++;	// increase the id for the next command

	// extract the next north and east commands
	float nextNorth = north;
	float nextEast = east;
	float nextAlt = alt;
	LOG_STATUS("sending command %d: N %f\tE %f\tA %f", nextCmd, nextNorth, nextEast, nextAlt);

	// build the mavlink message
	mavlink_tracking_cmd_t tracking_cmd;
	tracking_cmd.timestamp_usec = 0;
	tracking_cmd.north = nextNorth;
	tracking_cmd.east = nextEast;
	tracking_cmd.yaw_angle = 270.0;
	tracking_cmd.altitude = nextAlt;
	tracking_cmd.cmd_id = nextCmd;
	tracking_cmd.cmd_type = TRACKING_CMD_TRAVEL;

	mavlink_message_t message;
	mavlink_msg_tracking_cmd_encode(sysid, compid, &message, &tracking_cmd);

	/* printing stuff for debug purposes */
	int len = write_serial(message);
	if (len > 0 && _verbose) {
		LOG_STATUS("sending next tracking command");
	}

	return;
}


void MavlinkSerial::send_rotate_command(const float direction) {
	// retrieve the id of the last finished cmd
	int nextCmd = _command_id;
	_command_id++;	// increase the id for the next command

	// build the mavlink message
	mavlink_tracking_cmd_t tracking_cmd;
	tracking_cmd.timestamp_usec = 0;
	tracking_cmd.north = 0.0;			// don't travel any distance north
	tracking_cmd.east = 0.0;			// don't travel any distacne east
	tracking_cmd.yaw_angle = direction;		// rotate clockwise (NOTE: yaw angle no longer means yaw angle, but rather rotation direction)
	tracking_cmd.altitude = 0.0;		// will have pixhawk just use current altitude
	tracking_cmd.cmd_id = nextCmd;
	tracking_cmd.cmd_type = TRACKING_CMD_ROTATE;

	LOG_STATUS("sending rotate command (%d)\n", nextCmd);
	
	mavlink_message_t message;
	mavlink_msg_tracking_cmd_encode(sysid, compid, &message, &tracking_cmd);

	int len = write_serial(message);
	if (len > 0 && _verbose) {
		LOG_STATUS("sending next rotate command");
	}

	return;
}


void MavlinkSerial::send_finish_command() {

	// retrieve the id of the last finished cmd
	int nextCmd = _command_id++;

	// build the mavlink message
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

	int len = write_serial(message);
	if (len > 0 && _verbose) {
		LOG_STATUS("sending finish command");
	}
	// LOG_STATUS("Sent buffer of length %i\n",len);

	return;
}


void MavlinkSerial::send_bearing_cc_message(const double &bearing, const int32_t &lat, const int32_t &lon, const float &alt) {

	// build the mavlink message
	mavlink_bearing_cc_t bear;
	bear.bearing = bearing;
	bear.lat = lat;
	bear.lon = lon;
	bear.alt = alt;

	mavlink_message_t message;
	mavlink_msg_bearing_cc_encode(sysid, compid, &message, &bear);

	int len = write_serial(message);
	if (len > 0 && _verbose) {
		LOG_STATUS("sending bearing cc message");
	}

	return;
}

void MavlinkSerial::send_bearing_mle_message(const double &bearing, const int32_t &lat, const int32_t &lon, const float &alt) {

	// build the mavlink message
	mavlink_bearing_mle_t bear;
	bear.bearing = bearing;
	bear.lat = lat;
	bear.lon = lon;
	bear.alt = alt;

	mavlink_message_t message;
	mavlink_msg_bearing_mle_encode(sysid, compid, &message, &bear);

	int len = write_serial(message);
	if (len > 0 && _verbose) {
		LOG_STATUS("sending bearing mle message");
	}

	return;
}


void MavlinkSerial::send_rssi_message(const int &rssi, const int &rssi2, const int16_t &heading, const int32_t &lat, const int32_t &lon, const float &alt) {
	mavlink_rssi_t rssi_msg;
	rssi_msg.rssi_value = rssi;
	rssi_msg.rssi_value2 = rssi2;
	rssi_msg.heading = (float) heading;
	rssi_msg.lat = lat; //lat;
	rssi_msg.lon = lon; //lon;
	rssi_msg.alt = alt; // alt;

	mavlink_message_t message;
	mavlink_msg_rssi_encode(sysid, compid, &message, &rssi_msg);

	int len = write_serial(message);
	if (len > 0 && _verbose) {

		struct timeval tv;
		gettimeofday(&tv, NULL);
		unsigned long current_time = 1000000 * tv.tv_sec + tv.tv_usec;

		LOG_STATUS("%lu: sending rssi message", current_time);
	}
	
	return;
}
