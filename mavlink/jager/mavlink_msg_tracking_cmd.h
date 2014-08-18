// MESSAGE TRACKING_CMD PACKING

#define MAVLINK_MSG_ID_TRACKING_CMD 211

typedef struct __mavlink_tracking_cmd_t
{
 uint64_t timestamp_usec; ///< time since boot in usec of when the command was sent
 double north; ///< North distance to travel in meters
 double east; ///< East distance to travel in meters
 float yaw_angle; ///< Yaw angle through which to rotate, or the yaw angle in which to point the vehicle duing a travel command
 float altitude; ///< Altitude for the desired command
 uint16_t cmd_id; ///< id of this command, used for purposes of executing correct commands
 uint8_t cmd_type; ///< the type of the command, enumerated in TRACK_CMD
} mavlink_tracking_cmd_t;

#define MAVLINK_MSG_ID_TRACKING_CMD_LEN 35
#define MAVLINK_MSG_ID_211_LEN 35

#define MAVLINK_MSG_ID_TRACKING_CMD_CRC 62
#define MAVLINK_MSG_ID_211_CRC 62



#define MAVLINK_MESSAGE_INFO_TRACKING_CMD { \
	"TRACKING_CMD", \
	7, \
	{  { "timestamp_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tracking_cmd_t, timestamp_usec) }, \
         { "north", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_tracking_cmd_t, north) }, \
         { "east", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_tracking_cmd_t, east) }, \
         { "yaw_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_tracking_cmd_t, yaw_angle) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_tracking_cmd_t, altitude) }, \
         { "cmd_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_tracking_cmd_t, cmd_id) }, \
         { "cmd_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_tracking_cmd_t, cmd_type) }, \
         } \
}


/**
 * @brief Pack a tracking_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_usec time since boot in usec of when the command was sent
 * @param cmd_id id of this command, used for purposes of executing correct commands
 * @param cmd_type the type of the command, enumerated in TRACK_CMD
 * @param north North distance to travel in meters
 * @param east East distance to travel in meters
 * @param yaw_angle Yaw angle through which to rotate, or the yaw angle in which to point the vehicle duing a travel command
 * @param altitude Altitude for the desired command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracking_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp_usec, uint16_t cmd_id, uint8_t cmd_type, double north, double east, float yaw_angle, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKING_CMD_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_double(buf, 8, north);
	_mav_put_double(buf, 16, east);
	_mav_put_float(buf, 24, yaw_angle);
	_mav_put_float(buf, 28, altitude);
	_mav_put_uint16_t(buf, 32, cmd_id);
	_mav_put_uint8_t(buf, 34, cmd_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#else
	mavlink_tracking_cmd_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.north = north;
	packet.east = east;
	packet.yaw_angle = yaw_angle;
	packet.altitude = altitude;
	packet.cmd_id = cmd_id;
	packet.cmd_type = cmd_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRACKING_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRACKING_CMD_LEN, MAVLINK_MSG_ID_TRACKING_CMD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif
}

/**
 * @brief Pack a tracking_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_usec time since boot in usec of when the command was sent
 * @param cmd_id id of this command, used for purposes of executing correct commands
 * @param cmd_type the type of the command, enumerated in TRACK_CMD
 * @param north North distance to travel in meters
 * @param east East distance to travel in meters
 * @param yaw_angle Yaw angle through which to rotate, or the yaw angle in which to point the vehicle duing a travel command
 * @param altitude Altitude for the desired command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracking_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp_usec,uint16_t cmd_id,uint8_t cmd_type,double north,double east,float yaw_angle,float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKING_CMD_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_double(buf, 8, north);
	_mav_put_double(buf, 16, east);
	_mav_put_float(buf, 24, yaw_angle);
	_mav_put_float(buf, 28, altitude);
	_mav_put_uint16_t(buf, 32, cmd_id);
	_mav_put_uint8_t(buf, 34, cmd_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#else
	mavlink_tracking_cmd_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.north = north;
	packet.east = east;
	packet.yaw_angle = yaw_angle;
	packet.altitude = altitude;
	packet.cmd_id = cmd_id;
	packet.cmd_type = cmd_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRACKING_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRACKING_CMD_LEN, MAVLINK_MSG_ID_TRACKING_CMD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif
}

/**
 * @brief Encode a tracking_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tracking_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracking_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tracking_cmd_t* tracking_cmd)
{
	return mavlink_msg_tracking_cmd_pack(system_id, component_id, msg, tracking_cmd->timestamp_usec, tracking_cmd->cmd_id, tracking_cmd->cmd_type, tracking_cmd->north, tracking_cmd->east, tracking_cmd->yaw_angle, tracking_cmd->altitude);
}

/**
 * @brief Encode a tracking_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tracking_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracking_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tracking_cmd_t* tracking_cmd)
{
	return mavlink_msg_tracking_cmd_pack_chan(system_id, component_id, chan, msg, tracking_cmd->timestamp_usec, tracking_cmd->cmd_id, tracking_cmd->cmd_type, tracking_cmd->north, tracking_cmd->east, tracking_cmd->yaw_angle, tracking_cmd->altitude);
}

/**
 * @brief Send a tracking_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_usec time since boot in usec of when the command was sent
 * @param cmd_id id of this command, used for purposes of executing correct commands
 * @param cmd_type the type of the command, enumerated in TRACK_CMD
 * @param north North distance to travel in meters
 * @param east East distance to travel in meters
 * @param yaw_angle Yaw angle through which to rotate, or the yaw angle in which to point the vehicle duing a travel command
 * @param altitude Altitude for the desired command
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tracking_cmd_send(mavlink_channel_t chan, uint64_t timestamp_usec, uint16_t cmd_id, uint8_t cmd_type, double north, double east, float yaw_angle, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKING_CMD_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_double(buf, 8, north);
	_mav_put_double(buf, 16, east);
	_mav_put_float(buf, 24, yaw_angle);
	_mav_put_float(buf, 28, altitude);
	_mav_put_uint16_t(buf, 32, cmd_id);
	_mav_put_uint8_t(buf, 34, cmd_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_CMD, buf, MAVLINK_MSG_ID_TRACKING_CMD_LEN, MAVLINK_MSG_ID_TRACKING_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_CMD, buf, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif
#else
	mavlink_tracking_cmd_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.north = north;
	packet.east = east;
	packet.yaw_angle = yaw_angle;
	packet.altitude = altitude;
	packet.cmd_id = cmd_id;
	packet.cmd_type = cmd_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_CMD, (const char *)&packet, MAVLINK_MSG_ID_TRACKING_CMD_LEN, MAVLINK_MSG_ID_TRACKING_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_CMD, (const char *)&packet, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_TRACKING_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tracking_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_usec, uint16_t cmd_id, uint8_t cmd_type, double north, double east, float yaw_angle, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_double(buf, 8, north);
	_mav_put_double(buf, 16, east);
	_mav_put_float(buf, 24, yaw_angle);
	_mav_put_float(buf, 28, altitude);
	_mav_put_uint16_t(buf, 32, cmd_id);
	_mav_put_uint8_t(buf, 34, cmd_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_CMD, buf, MAVLINK_MSG_ID_TRACKING_CMD_LEN, MAVLINK_MSG_ID_TRACKING_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_CMD, buf, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif
#else
	mavlink_tracking_cmd_t *packet = (mavlink_tracking_cmd_t *)msgbuf;
	packet->timestamp_usec = timestamp_usec;
	packet->north = north;
	packet->east = east;
	packet->yaw_angle = yaw_angle;
	packet->altitude = altitude;
	packet->cmd_id = cmd_id;
	packet->cmd_type = cmd_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_CMD, (const char *)packet, MAVLINK_MSG_ID_TRACKING_CMD_LEN, MAVLINK_MSG_ID_TRACKING_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_CMD, (const char *)packet, MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE TRACKING_CMD UNPACKING


/**
 * @brief Get field timestamp_usec from tracking_cmd message
 *
 * @return time since boot in usec of when the command was sent
 */
static inline uint64_t mavlink_msg_tracking_cmd_get_timestamp_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field cmd_id from tracking_cmd message
 *
 * @return id of this command, used for purposes of executing correct commands
 */
static inline uint16_t mavlink_msg_tracking_cmd_get_cmd_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field cmd_type from tracking_cmd message
 *
 * @return the type of the command, enumerated in TRACK_CMD
 */
static inline uint8_t mavlink_msg_tracking_cmd_get_cmd_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field north from tracking_cmd message
 *
 * @return North distance to travel in meters
 */
static inline double mavlink_msg_tracking_cmd_get_north(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field east from tracking_cmd message
 *
 * @return East distance to travel in meters
 */
static inline double mavlink_msg_tracking_cmd_get_east(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field yaw_angle from tracking_cmd message
 *
 * @return Yaw angle through which to rotate, or the yaw angle in which to point the vehicle duing a travel command
 */
static inline float mavlink_msg_tracking_cmd_get_yaw_angle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field altitude from tracking_cmd message
 *
 * @return Altitude for the desired command
 */
static inline float mavlink_msg_tracking_cmd_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a tracking_cmd message into a struct
 *
 * @param msg The message to decode
 * @param tracking_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_tracking_cmd_decode(const mavlink_message_t* msg, mavlink_tracking_cmd_t* tracking_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP
	tracking_cmd->timestamp_usec = mavlink_msg_tracking_cmd_get_timestamp_usec(msg);
	tracking_cmd->north = mavlink_msg_tracking_cmd_get_north(msg);
	tracking_cmd->east = mavlink_msg_tracking_cmd_get_east(msg);
	tracking_cmd->yaw_angle = mavlink_msg_tracking_cmd_get_yaw_angle(msg);
	tracking_cmd->altitude = mavlink_msg_tracking_cmd_get_altitude(msg);
	tracking_cmd->cmd_id = mavlink_msg_tracking_cmd_get_cmd_id(msg);
	tracking_cmd->cmd_type = mavlink_msg_tracking_cmd_get_cmd_type(msg);
#else
	memcpy(tracking_cmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TRACKING_CMD_LEN);
#endif
}
