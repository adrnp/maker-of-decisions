// MESSAGE APNT_SITE_STATUS PACKING

#define MAVLINK_MSG_ID_APNT_SITE_STATUS 202

typedef struct __mavlink_apnt_site_status_t
{
 uint64_t timestamp_usec; ///< timestamp (microseoncds since UNIX) from apnt??
 uint32_t site_id[4]; ///< tarray of up to 4 site id's
 float site_lat[4]; ///< latitude of up to 4 sites seen by apnt
 float site_lon[4]; ///< longitude of up to 4 sites seen by apnt
 uint16_t site_signal[4]; ///< signal infomation for each of the up to 4 sites seen by apnt
} mavlink_apnt_site_status_t;

#define MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN 64
#define MAVLINK_MSG_ID_202_LEN 64

#define MAVLINK_MSG_ID_APNT_SITE_STATUS_CRC 58
#define MAVLINK_MSG_ID_202_CRC 58

#define MAVLINK_MSG_APNT_SITE_STATUS_FIELD_SITE_ID_LEN 4
#define MAVLINK_MSG_APNT_SITE_STATUS_FIELD_SITE_LAT_LEN 4
#define MAVLINK_MSG_APNT_SITE_STATUS_FIELD_SITE_LON_LEN 4
#define MAVLINK_MSG_APNT_SITE_STATUS_FIELD_SITE_SIGNAL_LEN 4

#define MAVLINK_MESSAGE_INFO_APNT_SITE_STATUS { \
	"APNT_SITE_STATUS", \
	5, \
	{  { "timestamp_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_apnt_site_status_t, timestamp_usec) }, \
         { "site_id", NULL, MAVLINK_TYPE_UINT32_T, 4, 8, offsetof(mavlink_apnt_site_status_t, site_id) }, \
         { "site_lat", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_apnt_site_status_t, site_lat) }, \
         { "site_lon", NULL, MAVLINK_TYPE_FLOAT, 4, 40, offsetof(mavlink_apnt_site_status_t, site_lon) }, \
         { "site_signal", NULL, MAVLINK_TYPE_UINT16_T, 4, 56, offsetof(mavlink_apnt_site_status_t, site_signal) }, \
         } \
}


/**
 * @brief Pack a apnt_site_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_usec timestamp (microseoncds since UNIX) from apnt??
 * @param site_id tarray of up to 4 site id's
 * @param site_lat latitude of up to 4 sites seen by apnt
 * @param site_lon longitude of up to 4 sites seen by apnt
 * @param site_signal signal infomation for each of the up to 4 sites seen by apnt
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_apnt_site_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp_usec, const uint32_t *site_id, const float *site_lat, const float *site_lon, const uint16_t *site_signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_uint32_t_array(buf, 8, site_id, 4);
	_mav_put_float_array(buf, 24, site_lat, 4);
	_mav_put_float_array(buf, 40, site_lon, 4);
	_mav_put_uint16_t_array(buf, 56, site_signal, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#else
	mavlink_apnt_site_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	mav_array_memcpy(packet.site_id, site_id, sizeof(uint32_t)*4);
	mav_array_memcpy(packet.site_lat, site_lat, sizeof(float)*4);
	mav_array_memcpy(packet.site_lon, site_lon, sizeof(float)*4);
	mav_array_memcpy(packet.site_signal, site_signal, sizeof(uint16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_APNT_SITE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN, MAVLINK_MSG_ID_APNT_SITE_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a apnt_site_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_usec timestamp (microseoncds since UNIX) from apnt??
 * @param site_id tarray of up to 4 site id's
 * @param site_lat latitude of up to 4 sites seen by apnt
 * @param site_lon longitude of up to 4 sites seen by apnt
 * @param site_signal signal infomation for each of the up to 4 sites seen by apnt
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_apnt_site_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp_usec,const uint32_t *site_id,const float *site_lat,const float *site_lon,const uint16_t *site_signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_uint32_t_array(buf, 8, site_id, 4);
	_mav_put_float_array(buf, 24, site_lat, 4);
	_mav_put_float_array(buf, 40, site_lon, 4);
	_mav_put_uint16_t_array(buf, 56, site_signal, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#else
	mavlink_apnt_site_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	mav_array_memcpy(packet.site_id, site_id, sizeof(uint32_t)*4);
	mav_array_memcpy(packet.site_lat, site_lat, sizeof(float)*4);
	mav_array_memcpy(packet.site_lon, site_lon, sizeof(float)*4);
	mav_array_memcpy(packet.site_signal, site_signal, sizeof(uint16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_APNT_SITE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN, MAVLINK_MSG_ID_APNT_SITE_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif
}

/**
 * @brief Encode a apnt_site_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param apnt_site_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_apnt_site_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_apnt_site_status_t* apnt_site_status)
{
	return mavlink_msg_apnt_site_status_pack(system_id, component_id, msg, apnt_site_status->timestamp_usec, apnt_site_status->site_id, apnt_site_status->site_lat, apnt_site_status->site_lon, apnt_site_status->site_signal);
}

/**
 * @brief Encode a apnt_site_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param apnt_site_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_apnt_site_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_apnt_site_status_t* apnt_site_status)
{
	return mavlink_msg_apnt_site_status_pack_chan(system_id, component_id, chan, msg, apnt_site_status->timestamp_usec, apnt_site_status->site_id, apnt_site_status->site_lat, apnt_site_status->site_lon, apnt_site_status->site_signal);
}

/**
 * @brief Send a apnt_site_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_usec timestamp (microseoncds since UNIX) from apnt??
 * @param site_id tarray of up to 4 site id's
 * @param site_lat latitude of up to 4 sites seen by apnt
 * @param site_lon longitude of up to 4 sites seen by apnt
 * @param site_signal signal infomation for each of the up to 4 sites seen by apnt
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_apnt_site_status_send(mavlink_channel_t chan, uint64_t timestamp_usec, const uint32_t *site_id, const float *site_lat, const float *site_lon, const uint16_t *site_signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_uint32_t_array(buf, 8, site_id, 4);
	_mav_put_float_array(buf, 24, site_lat, 4);
	_mav_put_float_array(buf, 40, site_lon, 4);
	_mav_put_uint16_t_array(buf, 56, site_signal, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_SITE_STATUS, buf, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN, MAVLINK_MSG_ID_APNT_SITE_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_SITE_STATUS, buf, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif
#else
	mavlink_apnt_site_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	mav_array_memcpy(packet.site_id, site_id, sizeof(uint32_t)*4);
	mav_array_memcpy(packet.site_lat, site_lat, sizeof(float)*4);
	mav_array_memcpy(packet.site_lon, site_lon, sizeof(float)*4);
	mav_array_memcpy(packet.site_signal, site_signal, sizeof(uint16_t)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_SITE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN, MAVLINK_MSG_ID_APNT_SITE_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_SITE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_apnt_site_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_usec, const uint32_t *site_id, const float *site_lat, const float *site_lon, const uint16_t *site_signal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_uint32_t_array(buf, 8, site_id, 4);
	_mav_put_float_array(buf, 24, site_lat, 4);
	_mav_put_float_array(buf, 40, site_lon, 4);
	_mav_put_uint16_t_array(buf, 56, site_signal, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_SITE_STATUS, buf, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN, MAVLINK_MSG_ID_APNT_SITE_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_SITE_STATUS, buf, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif
#else
	mavlink_apnt_site_status_t *packet = (mavlink_apnt_site_status_t *)msgbuf;
	packet->timestamp_usec = timestamp_usec;
	mav_array_memcpy(packet->site_id, site_id, sizeof(uint32_t)*4);
	mav_array_memcpy(packet->site_lat, site_lat, sizeof(float)*4);
	mav_array_memcpy(packet->site_lon, site_lon, sizeof(float)*4);
	mav_array_memcpy(packet->site_signal, site_signal, sizeof(uint16_t)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_SITE_STATUS, (const char *)packet, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN, MAVLINK_MSG_ID_APNT_SITE_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_SITE_STATUS, (const char *)packet, MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE APNT_SITE_STATUS UNPACKING


/**
 * @brief Get field timestamp_usec from apnt_site_status message
 *
 * @return timestamp (microseoncds since UNIX) from apnt??
 */
static inline uint64_t mavlink_msg_apnt_site_status_get_timestamp_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field site_id from apnt_site_status message
 *
 * @return tarray of up to 4 site id's
 */
static inline uint16_t mavlink_msg_apnt_site_status_get_site_id(const mavlink_message_t* msg, uint32_t *site_id)
{
	return _MAV_RETURN_uint32_t_array(msg, site_id, 4,  8);
}

/**
 * @brief Get field site_lat from apnt_site_status message
 *
 * @return latitude of up to 4 sites seen by apnt
 */
static inline uint16_t mavlink_msg_apnt_site_status_get_site_lat(const mavlink_message_t* msg, float *site_lat)
{
	return _MAV_RETURN_float_array(msg, site_lat, 4,  24);
}

/**
 * @brief Get field site_lon from apnt_site_status message
 *
 * @return longitude of up to 4 sites seen by apnt
 */
static inline uint16_t mavlink_msg_apnt_site_status_get_site_lon(const mavlink_message_t* msg, float *site_lon)
{
	return _MAV_RETURN_float_array(msg, site_lon, 4,  40);
}

/**
 * @brief Get field site_signal from apnt_site_status message
 *
 * @return signal infomation for each of the up to 4 sites seen by apnt
 */
static inline uint16_t mavlink_msg_apnt_site_status_get_site_signal(const mavlink_message_t* msg, uint16_t *site_signal)
{
	return _MAV_RETURN_uint16_t_array(msg, site_signal, 4,  56);
}

/**
 * @brief Decode a apnt_site_status message into a struct
 *
 * @param msg The message to decode
 * @param apnt_site_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_apnt_site_status_decode(const mavlink_message_t* msg, mavlink_apnt_site_status_t* apnt_site_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	apnt_site_status->timestamp_usec = mavlink_msg_apnt_site_status_get_timestamp_usec(msg);
	mavlink_msg_apnt_site_status_get_site_id(msg, apnt_site_status->site_id);
	mavlink_msg_apnt_site_status_get_site_lat(msg, apnt_site_status->site_lat);
	mavlink_msg_apnt_site_status_get_site_lon(msg, apnt_site_status->site_lon);
	mavlink_msg_apnt_site_status_get_site_signal(msg, apnt_site_status->site_signal);
#else
	memcpy(apnt_site_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_APNT_SITE_STATUS_LEN);
#endif
}
