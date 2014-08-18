#include <jager/mavlink.h>

// going to just use the mavlink data structures for things like gps info, site info, etc

/** struct containing information on the MAV we are currently connected to **/
struct MAVInfo {
	// this is general heartbeat information
	int systemId;
	int compId;
	int type;
	int autopilot;
	uint8_t base_mode;
	uint32_t custom_mode;
	int status;

	// apnt info (NOT NEEDED FOR LOUIS)
	mavlink_apnt_gps_status_t apnt_gps_status;
	mavlink_apnt_site_status_t apnt_site_status;

	// tracking info (NOT NEEDED FOR LOUIS)
	mavlink_tracking_cmd_t last_tracking_cmd;
	mavlink_tracking_status_t tracking_status;

	// cmd info
	uint16_t last_cmd_finished_id;
	uint16_t current_cmd_id;

	mavlink_highres_imu_t highres_imu;
	mavlink_attitude_t attitude;
	mavlink_vfr_hud_t vfr_hud;
	mavlink_global_position_int_t gps_position;
	mavlink_global_position_setpoint_int_t gps_setpoint;
	mavlink_roll_pitch_yaw_thrust_setpoint_t rpwt_setpoint;

	// useful battery info
	float battery_voltage;
	float battery_current;

	//TODO: add all the data interested in (all the custom jager mavlink messages)
};
