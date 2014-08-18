// MESSAGE TRACKING_STATUS PACKING

#define MAVLINK_MSG_ID_TRACKING_STATUS 210

typedef struct __mavlink_tracking_status_t
{
 uint64_t timestamp_usec; ///< time since boot in usec of last update from the tracking computer
 uint8_t computer_status; ///< current tracking status, see tracking status enum
 uint8_t hunt_mode_state; ///< current state of the hunt mode on the pixhawk, enumerated in hunt mode??
} mavlink_tracking_status_t;

#define MAVLINK_MSG_ID_TRACKING_STATUS_LEN 10
#define MAVLINK_MSG_ID_210_LEN 10

#define MAVLINK_MSG_ID_TRACKING_STATUS_CRC 8
#define MAVLINK_MSG_ID_210_CRC 8



#define MAVLINK_MESSAGE_INFO_TRACKING_STATUS { \
	"TRACKING_STATUS", \
	3, \
	{  { "timestamp_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tracking_status_t, timestamp_usec) }, \
         { "computer_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_tracking_status_t, computer_status) }, \
         { "hunt_mode_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_tracking_status_t, hunt_mode_state) }, \
         } \
}


/**
 * @brief Pack a tracking_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_usec time since boot in usec of last update from the tracking computer
 * @param computer_status current tracking status, see tracking status enum
 * @param hunt_mode_state current state of the hunt mode on the pixhawk, enumerated in hunt mode??
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracking_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp_usec, uint8_t computer_status, uint8_t hunt_mode_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKING_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_uint8_t(buf, 8, computer_status);
	_mav_put_uint8_t(buf, 9, hunt_mode_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#else
	mavlink_tracking_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.computer_status = computer_status;
	packet.hunt_mode_state = hunt_mode_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRACKING_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TRACKING_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif
}

/**
 * @brief Pack a tracking_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_usec time since boot in usec of last update from the tracking computer
 * @param computer_status current tracking status, see tracking status enum
 * @param hunt_mode_state current state of the hunt mode on the pixhawk, enumerated in hunt mode??
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracking_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp_usec,uint8_t computer_status,uint8_t hunt_mode_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKING_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_uint8_t(buf, 8, computer_status);
	_mav_put_uint8_t(buf, 9, hunt_mode_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#else
	mavlink_tracking_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.computer_status = computer_status;
	packet.hunt_mode_state = hunt_mode_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRACKING_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TRACKING_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif
}

/**
 * @brief Encode a tracking_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tracking_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracking_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tracking_status_t* tracking_status)
{
	return mavlink_msg_tracking_status_pack(system_id, component_id, msg, tracking_status->timestamp_usec, tracking_status->computer_status, tracking_status->hunt_mode_state);
}

/**
 * @brief Encode a tracking_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tracking_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracking_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tracking_status_t* tracking_status)
{
	return mavlink_msg_tracking_status_pack_chan(system_id, component_id, chan, msg, tracking_status->timestamp_usec, tracking_status->computer_status, tracking_status->hunt_mode_state);
}

/**
 * @brief Send a tracking_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_usec time since boot in usec of last update from the tracking computer
 * @param computer_status current tracking status, see tracking status enum
 * @param hunt_mode_state current state of the hunt mode on the pixhawk, enumerated in hunt mode??
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tracking_status_send(mavlink_channel_t chan, uint64_t timestamp_usec, uint8_t computer_status, uint8_t hunt_mode_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKING_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_uint8_t(buf, 8, computer_status);
	_mav_put_uint8_t(buf, 9, hunt_mode_state);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_STATUS, buf, MAVLINK_MSG_ID_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TRACKING_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_STATUS, buf, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif
#else
	mavlink_tracking_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.computer_status = computer_status;
	packet.hunt_mode_state = hunt_mode_state;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_STATUS, (const char *)&packet, MAVLINK_MSG_ID_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TRACKING_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_STATUS, (const char *)&packet, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_TRACKING_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tracking_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_usec, uint8_t computer_status, uint8_t hunt_mode_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_uint8_t(buf, 8, computer_status);
	_mav_put_uint8_t(buf, 9, hunt_mode_state);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_STATUS, buf, MAVLINK_MSG_ID_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TRACKING_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_STATUS, buf, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif
#else
	mavlink_tracking_status_t *packet = (mavlink_tracking_status_t *)msgbuf;
	packet->timestamp_usec = timestamp_usec;
	packet->computer_status = computer_status;
	packet->hunt_mode_state = hunt_mode_state;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_STATUS, (const char *)packet, MAVLINK_MSG_ID_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TRACKING_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_STATUS, (const char *)packet, MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE TRACKING_STATUS UNPACKING


/**
 * @brief Get field timestamp_usec from tracking_status message
 *
 * @return time since boot in usec of last update from the tracking computer
 */
static inline uint64_t mavlink_msg_tracking_status_get_timestamp_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field computer_status from tracking_status message
 *
 * @return current tracking status, see tracking status enum
 */
static inline uint8_t mavlink_msg_tracking_status_get_computer_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field hunt_mode_state from tracking_status message
 *
 * @return current state of the hunt mode on the pixhawk, enumerated in hunt mode??
 */
static inline uint8_t mavlink_msg_tracking_status_get_hunt_mode_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a tracking_status message into a struct
 *
 * @param msg The message to decode
 * @param tracking_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_tracking_status_decode(const mavlink_message_t* msg, mavlink_tracking_status_t* tracking_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	tracking_status->timestamp_usec = mavlink_msg_tracking_status_get_timestamp_usec(msg);
	tracking_status->computer_status = mavlink_msg_tracking_status_get_computer_status(msg);
	tracking_status->hunt_mode_state = mavlink_msg_tracking_status_get_hunt_mode_state(msg);
#else
	memcpy(tracking_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TRACKING_STATUS_LEN);
#endif
}
