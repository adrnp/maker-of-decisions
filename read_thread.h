/*
 * mavlink_thread.h
 *
 *  Created on: Aug 4, 2014
 *      Author: adrienp
 */

#ifndef READ_THREAD_H_
#define READ_THREAD_H_


// list of functions
void parse_heartbeat(const mavlink_message_t *message, MAVInfo *uavRead);

// default mavlink info
/*
void parse_sys_status(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_highres_imu(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_attitude(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_vfr_hud(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_global_position_int(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_gps_setpoint(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_rpwt_setpoint(const mavlink_message_t *message, MAVInfo *uavRead);

// custom mavlink info
void parse_apnt_gps_status(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_apnt_site_status(const mavlink_message_t *message, MAVInfo *uavRead);

void parse_tracking_cmd(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_tracking_status(const mavlink_message_t *message, MAVInfo *uavRead);

void parse_current_cmd_id(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_last_cmd_finished_id(const mavlink_message_t *message, MAVInfo *uavRead);
*/

// handle the incoming message (save data and trigger events as needed
void handle_message(const mavlink_message_t &message, MAVInfo *uavRead);


/* read from the serial port to get message. 
 *  Will return true if got message, false otherwise
 */
uint8_t read_from_serial(mavlink_status_t *lastStatus, mavlink_message_t *message);


// the most important function
void *read_thread(void *param);


#endif /* READ_THREAD_H_ */
