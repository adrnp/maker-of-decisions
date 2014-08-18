// MESSAGE HUNT_MISSION_REACHED PACKING

#define MAVLINK_MSG_ID_HUNT_MISSION_REACHED 213

typedef struct __mavlink_hunt_mission_reached_t
{
 uint16_t reached_cmd_id; ///< the cmd id of the command that was most recently completed
} mavlink_hunt_mission_reached_t;

#define MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN 2
#define MAVLINK_MSG_ID_213_LEN 2

#define MAVLINK_MSG_ID_HUNT_MISSION_REACHED_CRC 51
#define MAVLINK_MSG_ID_213_CRC 51



#define MAVLINK_MESSAGE_INFO_HUNT_MISSION_REACHED { \
	"HUNT_MISSION_REACHED", \
	1, \
	{  { "reached_cmd_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_hunt_mission_reached_t, reached_cmd_id) }, \
         } \
}


/**
 * @brief Pack a hunt_mission_reached message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param reached_cmd_id the cmd id of the command that was most recently completed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hunt_mission_reached_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t reached_cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN];
	_mav_put_uint16_t(buf, 0, reached_cmd_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#else
	mavlink_hunt_mission_reached_t packet;
	packet.reached_cmd_id = reached_cmd_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUNT_MISSION_REACHED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif
}

/**
 * @brief Pack a hunt_mission_reached message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reached_cmd_id the cmd id of the command that was most recently completed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hunt_mission_reached_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t reached_cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN];
	_mav_put_uint16_t(buf, 0, reached_cmd_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#else
	mavlink_hunt_mission_reached_t packet;
	packet.reached_cmd_id = reached_cmd_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUNT_MISSION_REACHED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif
}

/**
 * @brief Encode a hunt_mission_reached struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hunt_mission_reached C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hunt_mission_reached_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hunt_mission_reached_t* hunt_mission_reached)
{
	return mavlink_msg_hunt_mission_reached_pack(system_id, component_id, msg, hunt_mission_reached->reached_cmd_id);
}

/**
 * @brief Encode a hunt_mission_reached struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hunt_mission_reached C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hunt_mission_reached_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hunt_mission_reached_t* hunt_mission_reached)
{
	return mavlink_msg_hunt_mission_reached_pack_chan(system_id, component_id, chan, msg, hunt_mission_reached->reached_cmd_id);
}

/**
 * @brief Send a hunt_mission_reached message
 * @param chan MAVLink channel to send the message
 *
 * @param reached_cmd_id the cmd id of the command that was most recently completed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hunt_mission_reached_send(mavlink_channel_t chan, uint16_t reached_cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN];
	_mav_put_uint16_t(buf, 0, reached_cmd_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED, buf, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED, buf, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif
#else
	mavlink_hunt_mission_reached_t packet;
	packet.reached_cmd_id = reached_cmd_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED, (const char *)&packet, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED, (const char *)&packet, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hunt_mission_reached_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t reached_cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, reached_cmd_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED, buf, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED, buf, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif
#else
	mavlink_hunt_mission_reached_t *packet = (mavlink_hunt_mission_reached_t *)msgbuf;
	packet->reached_cmd_id = reached_cmd_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED, (const char *)packet, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUNT_MISSION_REACHED, (const char *)packet, MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE HUNT_MISSION_REACHED UNPACKING


/**
 * @brief Get field reached_cmd_id from hunt_mission_reached message
 *
 * @return the cmd id of the command that was most recently completed
 */
static inline uint16_t mavlink_msg_hunt_mission_reached_get_reached_cmd_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a hunt_mission_reached message into a struct
 *
 * @param msg The message to decode
 * @param hunt_mission_reached C-struct to decode the message contents into
 */
static inline void mavlink_msg_hunt_mission_reached_decode(const mavlink_message_t* msg, mavlink_hunt_mission_reached_t* hunt_mission_reached)
{
#if MAVLINK_NEED_BYTE_SWAP
	hunt_mission_reached->reached_cmd_id = mavlink_msg_hunt_mission_reached_get_reached_cmd_id(msg);
#else
	memcpy(hunt_mission_reached, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_HUNT_MISSION_REACHED_LEN);
#endif
}
