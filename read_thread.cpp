
#include "common_vars.h"
#include "common_include.h"
#include "read_thread.h"

using std::string;
using namespace std;


void parse_heartbeat(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_heartbeat_t heartbeat;

	mavlink_msg_heartbeat_decode(message, &heartbeat);

	uavRead->type = heartbeat.type;
	uavRead->autopilot = heartbeat.autopilot;
	uavRead->base_mode = heartbeat.base_mode;
	uavRead->custom_mode = heartbeat.custom_mode;
	uavRead->status = heartbeat.system_status;

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

void parse_highres_imu(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_msg_highres_imu_decode(message, &(uavRead->highres_imu));
}


void parse_attitude(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_msg_attitude_decode(message, &(uavRead->attitude));
}


void parse_vfr_hud(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_msg_vfr_hud_decode(message, &(uavRead->vfr_hud));
}


void parse_global_position_int(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_msg_global_position_int_decode(message, &(uavRead->gps_position));
}


void parse_gps_setpoint(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_msg_global_position_setpoint_int_decode(message, &(uavRead->gps_setpoint));
}


void parse_rpwt_setpoint(const mavlink_message_t *message, MAVInfo *uavRead) {
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(message, &(uavRead->rpwt_setpoint));
}



// custom mavlink parsers

void parse_apnt_gps_status(const mavlink_message_t *message, MAVInfo *uavRead) {
	// the new way of doing it
	mavlink_msg_apnt_gps_status_decode(message, &(uavRead->apnt_gps_status));
}

void parse_apnt_site_status(const mavlink_message_t *message, MAVInfo *uavRead) {
	// the new way of doing it
	mavlink_msg_apnt_site_status_decode(message, &(uavRead->apnt_site_status));
}

void parse_tracking_cmd(const mavlink_message_t *message, MAVInfo *uavRead) {
	// the new way of doing it
	mavlink_msg_tracking_cmd_decode(message, &(uavRead->last_tracking_cmd));

}

void parse_tracking_status(const mavlink_message_t *message, MAVInfo *uavRead) {
	// the new way of doing it
	mavlink_msg_tracking_status_decode(message, &(uavRead->tracking_status));
}

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
	if (lastCmd == hunt_mission_reached.reached_cmd_id) {
		uavRead->last_cmd_finished_id = hunt_mission_reached.reached_cmd_id;
	}

	// TODO: add call to generate command
	generateCommand();


}



/**
 * function to read from the serial device
 * right now only looks for the apnt_gps_status message and prints out
 * the current status (which happens to also be sent from this script)
 */
void *serial_read(void *param) {

	struct MAVInfo *uavRead = (struct MAVInfo *)param;


	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;

	// Blocking wait for new data
	while (!closeAll)
	{

		uint8_t cp;
		mavlink_message_t message;
		mavlink_status_t status;
		uint8_t msgReceived = false;

		if (read(fd, &cp, 1) > 0) {
			// Check if a message could be decoded, return the message in case yes
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
			if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
			{
				if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				if (debug)
				{
					unsigned char v=cp;
					fprintf(stderr,"%02x ", v);
				}
			}
			lastStatus = status;
		}
		else {
			if (!silent) fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
		}

		// If a message could be decoded, handle it
		// TODO: only need to do this once
		if(msgReceived) {

			if (!heartbeatReceived) {
				uavRead->systemId = message.sysid;
				uavRead->compId = message.compid;
			}


			switch (message.msgid)
			{
			case MAVLINK_MSG_ID_HEARTBEAT: // #0
			{
				parse_heartbeat(&message, uavRead);
				break;
			}
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
			case MAVLINK_MSG_ID_TRACKING_STATUS:
			{
				parse_tracking_status(&message, uavRead);

				break;
			}
			case MAVLINK_MSG_ID_TRACKING_CMD:
			{
				parse_tracking_cmd(&message, uavRead);
				break;
			}
			case MAVLINK_MSG_ID_HUNT_MISSION_CURRENT:
			{
				parse_current_cmd_id(&message, uavRead);
				break;
			}
			case MAVLINK_MSG_ID_HUNT_MISSION_REACHED:
			{
				parse_last_cmd_finished_id(&message, uavRead);
				break;
			}
			case MAVLINK_MSG_ID_SYS_STATUS:
			{
				parse_sys_status(&message, uavRead);
				break;
			} // TODO add all the needed stuff for louis
			case MAVLINK_MSG_ID_HIGHRES_IMU:
			{
				parse_highres_imu(&message, uavRead);
				break;
			}
			case MAVLINK_MSG_ID_ATTITUDE:
			{
				parse_attitude(&message, uavRead);
				break;
			}
			case MAVLINK_MSG_ID_VFR_HUD:
			{
				parse_vfr_hud(&message, uavRead);
				break;
			}
			case MAVLINK_MSG_ID_GPS_RAW_INT:
			{
				// XXX: not interesting for now
				break;
			}
			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			{
				parse_global_position_int(&message, uavRead);
				break;
			}
			case MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT:
			{
				parse_gps_setpoint(&message, uavRead);
				break;
			}
			case MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT:
			{
				parse_rpwt_setpoint(&message, uavRead);
				break;
			}
			} // end of switch
		}
	}

	cout << "read ending\n";
	return NULL;
}
