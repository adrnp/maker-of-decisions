// MESSAGE APNT_POSITION PACKING

#define MAVLINK_MSG_ID_APNT_POSITION 203

typedef struct __mavlink_apnt_position_t
{
 uint64_t timestamp_usec; ///< timestamp since boot in usec of the position information from apnt
 float apnt_lat; ///< latitude from apnt
 float apnt_lon; ///< longitude from apnt
} mavlink_apnt_position_t;

#define MAVLINK_MSG_ID_APNT_POSITION_LEN 16
#define MAVLINK_MSG_ID_203_LEN 16

#define MAVLINK_MSG_ID_APNT_POSITION_CRC 168
#define MAVLINK_MSG_ID_203_CRC 168



#define MAVLINK_MESSAGE_INFO_APNT_POSITION { \
	"APNT_POSITION", \
	3, \
	{  { "timestamp_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_apnt_position_t, timestamp_usec) }, \
         { "apnt_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_apnt_position_t, apnt_lat) }, \
         { "apnt_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_apnt_position_t, apnt_lon) }, \
         } \
}


/**
 * @brief Pack a apnt_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_usec timestamp since boot in usec of the position information from apnt
 * @param apnt_lat latitude from apnt
 * @param apnt_lon longitude from apnt
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_apnt_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp_usec, float apnt_lat, float apnt_lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_POSITION_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_float(buf, 8, apnt_lat);
	_mav_put_float(buf, 12, apnt_lon);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#else
	mavlink_apnt_position_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.apnt_lat = apnt_lat;
	packet.apnt_lon = apnt_lon;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_APNT_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_APNT_POSITION_LEN, MAVLINK_MSG_ID_APNT_POSITION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif
}

/**
 * @brief Pack a apnt_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_usec timestamp since boot in usec of the position information from apnt
 * @param apnt_lat latitude from apnt
 * @param apnt_lon longitude from apnt
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_apnt_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp_usec,float apnt_lat,float apnt_lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_POSITION_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_float(buf, 8, apnt_lat);
	_mav_put_float(buf, 12, apnt_lon);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#else
	mavlink_apnt_position_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.apnt_lat = apnt_lat;
	packet.apnt_lon = apnt_lon;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_APNT_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_APNT_POSITION_LEN, MAVLINK_MSG_ID_APNT_POSITION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif
}

/**
 * @brief Encode a apnt_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param apnt_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_apnt_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_apnt_position_t* apnt_position)
{
	return mavlink_msg_apnt_position_pack(system_id, component_id, msg, apnt_position->timestamp_usec, apnt_position->apnt_lat, apnt_position->apnt_lon);
}

/**
 * @brief Encode a apnt_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param apnt_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_apnt_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_apnt_position_t* apnt_position)
{
	return mavlink_msg_apnt_position_pack_chan(system_id, component_id, chan, msg, apnt_position->timestamp_usec, apnt_position->apnt_lat, apnt_position->apnt_lon);
}

/**
 * @brief Send a apnt_position message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_usec timestamp since boot in usec of the position information from apnt
 * @param apnt_lat latitude from apnt
 * @param apnt_lon longitude from apnt
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_apnt_position_send(mavlink_channel_t chan, uint64_t timestamp_usec, float apnt_lat, float apnt_lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_POSITION_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_float(buf, 8, apnt_lat);
	_mav_put_float(buf, 12, apnt_lon);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_POSITION, buf, MAVLINK_MSG_ID_APNT_POSITION_LEN, MAVLINK_MSG_ID_APNT_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_POSITION, buf, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif
#else
	mavlink_apnt_position_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.apnt_lat = apnt_lat;
	packet.apnt_lon = apnt_lon;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_POSITION, (const char *)&packet, MAVLINK_MSG_ID_APNT_POSITION_LEN, MAVLINK_MSG_ID_APNT_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_POSITION, (const char *)&packet, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_APNT_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_apnt_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_usec, float apnt_lat, float apnt_lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_float(buf, 8, apnt_lat);
	_mav_put_float(buf, 12, apnt_lon);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_POSITION, buf, MAVLINK_MSG_ID_APNT_POSITION_LEN, MAVLINK_MSG_ID_APNT_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_POSITION, buf, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif
#else
	mavlink_apnt_position_t *packet = (mavlink_apnt_position_t *)msgbuf;
	packet->timestamp_usec = timestamp_usec;
	packet->apnt_lat = apnt_lat;
	packet->apnt_lon = apnt_lon;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_POSITION, (const char *)packet, MAVLINK_MSG_ID_APNT_POSITION_LEN, MAVLINK_MSG_ID_APNT_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_POSITION, (const char *)packet, MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE APNT_POSITION UNPACKING


/**
 * @brief Get field timestamp_usec from apnt_position message
 *
 * @return timestamp since boot in usec of the position information from apnt
 */
static inline uint64_t mavlink_msg_apnt_position_get_timestamp_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field apnt_lat from apnt_position message
 *
 * @return latitude from apnt
 */
static inline float mavlink_msg_apnt_position_get_apnt_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field apnt_lon from apnt_position message
 *
 * @return longitude from apnt
 */
static inline float mavlink_msg_apnt_position_get_apnt_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a apnt_position message into a struct
 *
 * @param msg The message to decode
 * @param apnt_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_apnt_position_decode(const mavlink_message_t* msg, mavlink_apnt_position_t* apnt_position)
{
#if MAVLINK_NEED_BYTE_SWAP
	apnt_position->timestamp_usec = mavlink_msg_apnt_position_get_timestamp_usec(msg);
	apnt_position->apnt_lat = mavlink_msg_apnt_position_get_apnt_lat(msg);
	apnt_position->apnt_lon = mavlink_msg_apnt_position_get_apnt_lon(msg);
#else
	memcpy(apnt_position, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_APNT_POSITION_LEN);
#endif
}
