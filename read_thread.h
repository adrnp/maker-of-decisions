/**
 * @file read_thread.h
 *
 * Definitions for the thread that reads the pixhawk data.
 * Sets all pixhawk data to the common mav struct to be used by other threads.
 * 
 * @Author Adrien Perkins <adrienp@stanford.edu>
 */


#ifndef READ_THREAD_H_
#define READ_THREAD_H_


 /**
  * parse the heartbeat message.
  * @param message  the mavlink message
  * @param uavRead  pointer to the common struct where to save the data
  */
void parse_heartbeat(const mavlink_message_t *message, MAVInfo *uavRead);

/**
 * parse the system status message.
 * @param message  the mavlink message
 * @param uavRead  pointer to the common struct where to save the data
 */
void parse_sys_status(const mavlink_message_t *message, MAVInfo *uavRead);

/**
 * parse the message containing the current command being processed.
 * @param message  the mavlink message
 * @param uavRead  pointer to the common struct where to save the data
 */
void parse_current_cmd_id(const mavlink_message_t *message, MAVInfo *uavRead);

/**
 * parse the message containing the last command finished.
 * @param message  the mavlink message
 * @param uavRead  pointer to the common struct where to save the data
 */
void parse_last_cmd_finished_id(const mavlink_message_t *message, MAVInfo *uavRead);

// custom mavlink info
/*
void parse_apnt_gps_status(const mavlink_message_t *message, MAVInfo *uavRead);
void parse_apnt_site_status(const mavlink_message_t *message, MAVInfo *uavRead);
*/

/**
 * handle the incoming message (save data and trigger events as needed)
 * @param message  the incoming message
 * @param uavRead  pointer to the common struct where to save the data
 */
void handle_message(const mavlink_message_t &message, MAVInfo *uavRead);

/**
 * the main function for the thread.
 * @param  param  the mav struct
 * @return        0 on termination
 */
void *read_thread(void *param);


#endif /* READ_THREAD_H_ */
