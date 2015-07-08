/*
 * mavlink_thread.h
 *
 *  Created on: Aug 4, 2014
 *      Author: adrienp
 */

#ifndef READ_THREAD_H_
#define READ_THREAD_H_


// list of functions

 // default mavlink messages that aren't strait forward parses
void parse_heartbeat(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_sys_status(const mavlink_message_t *message, MAVInfo *uavRead);

// custom tracking specifc cmds
void parse_current_cmd_id(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_last_cmd_finished_id(const mavlink_message_t *message, MAVInfo *uavRead);



// custom mavlink info
/*
void parse_apnt_gps_status(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_apnt_site_status(const mavlink_message_t *message, MAVInfo *uavRead);
*/



// handle the incoming message (save data and trigger events as needed
void handle_message(const mavlink_message_t &message, MAVInfo *uavRead);


// the most important function
void *read_thread(void *param);


#endif /* READ_THREAD_H_ */
