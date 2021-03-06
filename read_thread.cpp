/**
 * @file read_thread.cpp
 * 
 * Reads data from the pixhawk over mavlink, parsing the incoming messages
 * and updating the shared MAVInfo object about JAGER.
 *
 * Currently only handles a small subset of the messages from JAGER.
 * (only the ones needed)
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
#include "commander.h"

using std::string;
using namespace std;

/** whether or not we have received a heartbeat from the pixhawk */
bool heartbeatReceived = false;

/** whether or not we are currently in hunt mode */
bool hunting = false;

/* whether or not we are currently rotating */
// bool rotating = false;

/** keep track of the previous hunt state, as hunt state is sent periodically, not just on updates */
int prev_hs = -1;

/** keep track of whether or not the mode has changed */
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

	if (!heartbeatReceived) {
		LOG_STATUS("[READ] Received vehicle heartbeat");
		LOG_STATUS("[READ] The system id: %u", message->sysid);
		LOG_STATUS("[READ] The component id: %u", message->compid);
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

	//LOG_DEBUG("[READ] received message (%d)", message->msgid);

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

			LOG_DEBUG("[READ] HUNT STATE is currently %d", (int) uavRead->tracking_status.hunt_mode_state);

			if (uavRead->tracking_status.hunt_mode_state == prev_hs && !mode_change) {
				break;
			}

			// set this hunt state as the previous hunt state
			prev_hs = uavRead->tracking_status.hunt_mode_state;

			LOG_STATUS("[READ] HUNT STATE changed to: %d", (int) uavRead->tracking_status.hunt_mode_state);

			// need to check to see if we have changed into waiting for the first time
			if (!hunting && uavRead->tracking_status.hunt_mode_state > TRACKING_HUNT_STATE_OFF) {
				hunting = true;
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
		case MAVLINK_MSG_ID_SYSTEM_TIME_USEC:
		{
			mavlink_msg_system_time_usec_decode(message, &(uavRead->sys_time_us));
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
	while (common::RUNNING_FLAG) {

		// variables needed for the message reading
		mavlink_message_t message;	// the message itself
		
		// read from serial, if message received, will be written to message variable
		uint8_t msgReceived = common::pixhawk->read_serial(&lastStatus, &message);

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

	LOG_STATUS("[READ] read ending");
	return NULL;
}
