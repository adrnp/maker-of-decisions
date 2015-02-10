/**
* @file mav_struct.h
* 
* this file constants the structure that contains the vital
* information about the UAV.  Basically just a copy of the 
* information known by the Pixhawk.
*
*/

// include the mavlink message definitions, since using structures from mavlink definitions
#include <jager/mavlink.h>


/** struct containing information on the MAV we are currently connected to **/
struct MAVInfo {

	// this is general heartbeat information
	int systemId;			// the vehicle's system ID
	int compId;				// the autopilot's component ID
	int type;				// the type of vehicle it is (enumerated in one of the mavlink files, TODO: find which one)
	int autopilot;			//	not sure
	uint8_t base_mode;		// the base control mode the autopilot is in
	uint32_t custom_mode;	// the custom control mode the autopilot is in (together with base mode, can determine the human readible mode)
	int status;				// the status of the vehicle (enumerated in a mavlink file, TODO: find which one)

	// apnt info (NOT NEEDED FOR LOUIS)
	// mavlink_apnt_gps_status_t apnt_gps_status;
	// mavlink_apnt_site_status_t apnt_site_status;

	// tracking info (NOT NEEDED FOR LOUIS)
	// mavlink_tracking_cmd_t last_tracking_cmd;
	// mavlink_tracking_status_t tracking_status;

	// cmd info
	// uint16_t last_cmd_finished_id;
	// uint16_t current_cmd_id;

	// structures that contain information on the attitude of the vehicle
	mavlink_highres_imu_t highres_imu;
	mavlink_attitude_t attitude;
	mavlink_vfr_hud_t vfr_hud;

	
	// structures that contain setpoint information (attitude and position)
	// setpoints are targets the vehicle is trying to hit (e.g. attitude setpoint is the desired vehicle attitude)
	mavlink_attitude_target_t attitude_targer;
	mavlink_position_target_global_int_t position_target_gps;

	// structures that contain information on position (raw, filtered and NED positions)
	mavlink_gps_raw_int_t gps_raw;
	mavlink_global_position_int_t gps_position;
	mavlink_local_position_ned_t local_ned;


	// useful battery info
	float battery_voltage;
	float battery_current;

	//TODO: add all the data interested in (all the custom jager mavlink messages)
};
