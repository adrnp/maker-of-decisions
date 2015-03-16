
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
#
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>


#include "common.h"
#include "read_thread.h"
#include "serial_port.h"
#include "commander.h"

using std::string;
using namespace std;

/* whether or not we have received a heartbeat from the pixhawk */
bool heartbeatReceived = false;

/* whether or not we are currently in hunt mode */
bool hunting = false;

/* whether or not we are currently rotating */
bool rotating = false;

/* keep track of the previous hunt state, as hunt state is sent periodically, not just on updates */
int prev_hunt_state = -1;

/* keep track of whether or not the mode has changed */
bool mode_change = false;


void parse_heartbeat(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_heartbeat_t heartbeat;

	mavlink_msg_heartbeat_decode(message, &heartbeat);

	uavRead->type = heartbeat.type;
	uavRead->autopilot = heartbeat.autopilot;
	uavRead->base_mode = heartbeat.base_mode;
	// uavRead->custom_mode = heartbeat.custom_mode;
	uavRead->status = heartbeat.system_status;

	if (heartbeat.custom_mode != uavRead->custom_mode) {
		mode_change = true;
		uavRead->custom_mode = heartbeat.custom_mode;
	}

	// so we don't keep rewriting known values that stay constant
	// realizing not really doing anything with this....
	heartbeatReceived = true;
}


void parse_sys_status(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_sys_status_t sysStatus;
	mavlink_msg_sys_status_decode(message, &sysStatus);

	uavRead->battery_voltage = (float) sysStatus.voltage_battery/1000.0f;
	uavRead->battery_current = (float) sysStatus.current_battery/100.0f;
}


// custom mavlink parsers
/*

void parse_apnt_gps_status(const mavlink_message_t *message, MAVInfo *uavRead) {
	// the new way of doing it
	mavlink_msg_apnt_gps_status_decode(message, &(uavRead->apnt_gps_status));
}

void parse_apnt_site_status(const mavlink_message_t *message, MAVInfo *uavRead) {
	// the new way of doing it
	mavlink_msg_apnt_site_status_decode(message, &(uavRead->apnt_site_status));
}

*/

void parse_current_cmd_id(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_hunt_mission_current_t hunt_mission_current;
	mavlink_msg_hunt_mission_current_decode(message, &hunt_mission_current);

	uavRead->current_cmd_id = hunt_mission_current.current_cmd_id;
}

void parse_last_cmd_finished_id(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_hunt_mission_reached_t hunt_mission_reached;
	mavlink_msg_hunt_mission_reached_decode(message, &hunt_mission_reached);

	// only update this if this confirms the last command sent (anything else will be assumed to be
	// delayed or just wrong)
	if (uavRead->current_cmd_id == hunt_mission_reached.reached_cmd_id) {
		uavRead->last_cmd_finished_id = hunt_mission_reached.reached_cmd_id;
	}

	// TODO: add call to generate command
	// sendNextCommand();


}


void handle_message(const mavlink_message_t *message, MAVInfo *uavRead) {


	switch (message->msgid)
	{
		//normal messages
		case MAVLINK_MSG_ID_HEARTBEAT: // #0
		{
			parse_heartbeat(message, uavRead);
			break;
		}
		case MAVLINK_MSG_ID_SYS_STATUS:
		{
			parse_sys_status(message, uavRead);
			break;
		}
		case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
			mavlink_msg_highres_imu_decode(message, &(uavRead->highres_imu));
			break;
		}
		case MAVLINK_MSG_ID_ATTITUDE:
		{
			mavlink_msg_attitude_decode(message, &(uavRead->attitude));

			// cout << "heading: " << uavRead->attitude.yaw*180/3.14 << "\n";
			break;
		}
		case MAVLINK_MSG_ID_VFR_HUD:
		{
			mavlink_msg_vfr_hud_decode(message, &(uavRead->vfr_hud));

			// cout << "heading: " << uavRead->vfr_hud.heading << "\n";
			break;
		}
		
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
			mavlink_msg_global_position_int_decode(message, &(uavRead->gps_position));
			break;
		}
		case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
		{
			mavlink_msg_position_target_global_int_decode(message, &(uavRead->position_target_gps));
			break;
		}
		case MAVLINK_MSG_ID_ATTITUDE_TARGET:
		{
			mavlink_msg_attitude_target_decode(message, &(uavRead->attitude_target));
			break;
		} 
		
		// tracking specific messages
		case MAVLINK_MSG_ID_TRACKING_STATUS:
		{
			mavlink_msg_tracking_status_decode(message, &(uavRead->tracking_status));

			if (uavRead->tracking_status.hunt_mode_state == prev_hunt_state && !mode_change) {
				break;
			}

			// set this hunt state as the previous hunt state
			prev_hunt_state = uavRead->tracking_status.hunt_mode_state;

			cout << "HUNT STATE changed to: " << (int) uavRead->tracking_status.hunt_mode_state << "\n";

			// need to check to see if we have changed into waiting for the first time
			if (!hunting && uavRead->tracking_status.hunt_mode_state > TRACKING_HUNT_STATE_OFF) {
				hunting = true;

				// this is the first time we have triggered into hunting
				uavRead->last_cmd_finished_id = -1; // set the last cmd id to -1, so that when we run get next cmd it sends the correct one

				// command the vehicle to rotate
				sendRotateCommand();

				// mark that we are now rotating
				rotating = true;
			}

			/* if the pixhawk is in wait mode, send a rotate command */
			if (!rotating && uavRead->tracking_status.hunt_mode_state == TRACKING_HUNT_STATE_WAIT) {
				sendRotateCommand();

				// mark that we are now rotating
				rotating = true;
			} else {
				// finishing the rotation
				rotating = false;

				// TODO: this is where we will want to calculate the bearing....
				// NOTE: wifly thread is currently doing bearing calculations
			}

			mode_change = false;



			break;
		}
		case MAVLINK_MSG_ID_TRACKING_CMD:
		{
			mavlink_msg_tracking_cmd_decode(message, &(uavRead->last_tracking_cmd));
			break;
		}
		case MAVLINK_MSG_ID_HUNT_MISSION_CURRENT:
		{
			parse_current_cmd_id(message, uavRead);
			break;
		}
		case MAVLINK_MSG_ID_HUNT_MISSION_REACHED:
		{
			parse_last_cmd_finished_id(message, uavRead);
			break;
		}
		 // TODO add all the needed stuff for louis

		/* // COMMENTING OUT APNT MESSAGES FOR NOW
		case MAVLINK_MSG_ID_APNT_GPS_STATUS:
		{
			parse_apnt_gps_status(&message, uavRead);
			break;
		}
		case MAVLINK_MSG_ID_APNT_SITE_STATUS:
		{
			parse_apnt_site_status(&message, uavRead);
			break;
		}
		*/
	} // end of switch
}


uint8_t read_from_serial(mavlink_status_t *lastStatus, mavlink_message_t *message) {
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
			if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			
			// print out the characters of the packets themselves
			if (debug) {
				unsigned char v=cp;
				fprintf(stderr,"%02x ", v);
			}
		}

		// update the last message status 
		*lastStatus = status;

	} else { // means unable to read from the serial device

		// print out error as needed
		if (verbose) fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
	}

	// return whether or not the message was received
	return msgReceived;
}




/**
 * function to read from the serial device
 * right now only looks for the apnt_gps_status message and prints out
 * the current status (which happens to also be sent from this script)
 */
void *read_thread(void *param) {

	// retrieve the MAVInfo struct that is sent to this function as a parameter
	struct MAVInfo *uavRead = (struct MAVInfo *)param;


	// variables to hold information about the connection health
	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;	// initialize to 0 just for the first pass

	// run this infinite loop (as long as RUNNING_FLAG == 1) which reads messages as they come in
	while (RUNNING_FLAG) {

		// variables needed for the message reading
		mavlink_message_t message;	// the message itself
		
		// read from serial, if message received, will be written to message variable
		uint8_t msgReceived = read_from_serial(&lastStatus, &message);

		// If a message could be decoded, handle it
		// TODO: only need to do this once
		if(msgReceived) {

			if (!heartbeatReceived) {
				uavRead->systemId = message.sysid;
				uavRead->compId = message.compid;
			}

			// handle what need to happen with the message
			handle_message(&message, uavRead);	
		}
	}

	cout << "read ending\n";
	return NULL;
}
