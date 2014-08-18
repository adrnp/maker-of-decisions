// MESSAGE APNT_GPS_STATUS PACKING

#define MAVLINK_MSG_ID_APNT_GPS_STATUS 201

typedef struct __mavlink_apnt_gps_status_t
{
 uint64_t timestamp_usec; ///< timestamp (microseconds since UNIX) from gps?
 float gps_lat; ///< latitude from apnt gps
 float gps_lon; ///< longitude from apnt gps
 float gps_alt; ///< altitude from apnt gps
 int16_t azimuth[8]; ///< array of the azimuth of up to 8 satellites in view
 uint8_t prn[8]; ///< array of prn code of up to 8 satellites in view
 uint8_t elevation[8]; ///< array of the elevation of up to 8 satellites in view
 uint8_t snr[8]; ///< array of signal to noise ration of up to 8 satellites in view
} mavlink_apnt_gps_status_t;

#define MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN 60
#define MAVLINK_MSG_ID_201_LEN 60

#define MAVLINK_MSG_ID_APNT_GPS_STATUS_CRC 225
#define MAVLINK_MSG_ID_201_CRC 225

#define MAVLINK_MSG_APNT_GPS_STATUS_FIELD_AZIMUTH_LEN 8
#define MAVLINK_MSG_APNT_GPS_STATUS_FIELD_PRN_LEN 8
#define MAVLINK_MSG_APNT_GPS_STATUS_FIELD_ELEVATION_LEN 8
#define MAVLINK_MSG_APNT_GPS_STATUS_FIELD_SNR_LEN 8

#define MAVLINK_MESSAGE_INFO_APNT_GPS_STATUS { \
	"APNT_GPS_STATUS", \
	8, \
	{  { "timestamp_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_apnt_gps_status_t, timestamp_usec) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_apnt_gps_status_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_apnt_gps_status_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_apnt_gps_status_t, gps_alt) }, \
         { "azimuth", NULL, MAVLINK_TYPE_INT16_T, 8, 20, offsetof(mavlink_apnt_gps_status_t, azimuth) }, \
         { "prn", NULL, MAVLINK_TYPE_UINT8_T, 8, 36, offsetof(mavlink_apnt_gps_status_t, prn) }, \
         { "elevation", NULL, MAVLINK_TYPE_UINT8_T, 8, 44, offsetof(mavlink_apnt_gps_status_t, elevation) }, \
         { "snr", NULL, MAVLINK_TYPE_UINT8_T, 8, 52, offsetof(mavlink_apnt_gps_status_t, snr) }, \
         } \
}


/**
 * @brief Pack a apnt_gps_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_usec timestamp (microseconds since UNIX) from gps?
 * @param gps_lat latitude from apnt gps
 * @param gps_lon longitude from apnt gps
 * @param gps_alt altitude from apnt gps
 * @param prn array of prn code of up to 8 satellites in view
 * @param azimuth array of the azimuth of up to 8 satellites in view
 * @param elevation array of the elevation of up to 8 satellites in view
 * @param snr array of signal to noise ration of up to 8 satellites in view
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_apnt_gps_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp_usec, float gps_lat, float gps_lon, float gps_alt, const uint8_t *prn, const int16_t *azimuth, const uint8_t *elevation, const uint8_t *snr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_float(buf, 8, gps_lat);
	_mav_put_float(buf, 12, gps_lon);
	_mav_put_float(buf, 16, gps_alt);
	_mav_put_int16_t_array(buf, 20, azimuth, 8);
	_mav_put_uint8_t_array(buf, 36, prn, 8);
	_mav_put_uint8_t_array(buf, 44, elevation, 8);
	_mav_put_uint8_t_array(buf, 52, snr, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#else
	mavlink_apnt_gps_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.gps_lat = gps_lat;
	packet.gps_lon = gps_lon;
	packet.gps_alt = gps_alt;
	mav_array_memcpy(packet.azimuth, azimuth, sizeof(int16_t)*8);
	mav_array_memcpy(packet.prn, prn, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.elevation, elevation, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.snr, snr, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_APNT_GPS_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN, MAVLINK_MSG_ID_APNT_GPS_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif
}

/**
 * @brief Pack a apnt_gps_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_usec timestamp (microseconds since UNIX) from gps?
 * @param gps_lat latitude from apnt gps
 * @param gps_lon longitude from apnt gps
 * @param gps_alt altitude from apnt gps
 * @param prn array of prn code of up to 8 satellites in view
 * @param azimuth array of the azimuth of up to 8 satellites in view
 * @param elevation array of the elevation of up to 8 satellites in view
 * @param snr array of signal to noise ration of up to 8 satellites in view
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_apnt_gps_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp_usec,float gps_lat,float gps_lon,float gps_alt,const uint8_t *prn,const int16_t *azimuth,const uint8_t *elevation,const uint8_t *snr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_float(buf, 8, gps_lat);
	_mav_put_float(buf, 12, gps_lon);
	_mav_put_float(buf, 16, gps_alt);
	_mav_put_int16_t_array(buf, 20, azimuth, 8);
	_mav_put_uint8_t_array(buf, 36, prn, 8);
	_mav_put_uint8_t_array(buf, 44, elevation, 8);
	_mav_put_uint8_t_array(buf, 52, snr, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#else
	mavlink_apnt_gps_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.gps_lat = gps_lat;
	packet.gps_lon = gps_lon;
	packet.gps_alt = gps_alt;
	mav_array_memcpy(packet.azimuth, azimuth, sizeof(int16_t)*8);
	mav_array_memcpy(packet.prn, prn, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.elevation, elevation, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.snr, snr, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_APNT_GPS_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN, MAVLINK_MSG_ID_APNT_GPS_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif
}

/**
 * @brief Encode a apnt_gps_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param apnt_gps_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_apnt_gps_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_apnt_gps_status_t* apnt_gps_status)
{
	return mavlink_msg_apnt_gps_status_pack(system_id, component_id, msg, apnt_gps_status->timestamp_usec, apnt_gps_status->gps_lat, apnt_gps_status->gps_lon, apnt_gps_status->gps_alt, apnt_gps_status->prn, apnt_gps_status->azimuth, apnt_gps_status->elevation, apnt_gps_status->snr);
}

/**
 * @brief Encode a apnt_gps_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param apnt_gps_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_apnt_gps_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_apnt_gps_status_t* apnt_gps_status)
{
	return mavlink_msg_apnt_gps_status_pack_chan(system_id, component_id, chan, msg, apnt_gps_status->timestamp_usec, apnt_gps_status->gps_lat, apnt_gps_status->gps_lon, apnt_gps_status->gps_alt, apnt_gps_status->prn, apnt_gps_status->azimuth, apnt_gps_status->elevation, apnt_gps_status->snr);
}

/**
 * @brief Send a apnt_gps_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_usec timestamp (microseconds since UNIX) from gps?
 * @param gps_lat latitude from apnt gps
 * @param gps_lon longitude from apnt gps
 * @param gps_alt altitude from apnt gps
 * @param prn array of prn code of up to 8 satellites in view
 * @param azimuth array of the azimuth of up to 8 satellites in view
 * @param elevation array of the elevation of up to 8 satellites in view
 * @param snr array of signal to noise ration of up to 8 satellites in view
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_apnt_gps_status_send(mavlink_channel_t chan, uint64_t timestamp_usec, float gps_lat, float gps_lon, float gps_alt, const uint8_t *prn, const int16_t *azimuth, const uint8_t *elevation, const uint8_t *snr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_float(buf, 8, gps_lat);
	_mav_put_float(buf, 12, gps_lon);
	_mav_put_float(buf, 16, gps_alt);
	_mav_put_int16_t_array(buf, 20, azimuth, 8);
	_mav_put_uint8_t_array(buf, 36, prn, 8);
	_mav_put_uint8_t_array(buf, 44, elevation, 8);
	_mav_put_uint8_t_array(buf, 52, snr, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_GPS_STATUS, buf, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN, MAVLINK_MSG_ID_APNT_GPS_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_GPS_STATUS, buf, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif
#else
	mavlink_apnt_gps_status_t packet;
	packet.timestamp_usec = timestamp_usec;
	packet.gps_lat = gps_lat;
	packet.gps_lon = gps_lon;
	packet.gps_alt = gps_alt;
	mav_array_memcpy(packet.azimuth, azimuth, sizeof(int16_t)*8);
	mav_array_memcpy(packet.prn, prn, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.elevation, elevation, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.snr, snr, sizeof(uint8_t)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_GPS_STATUS, (const char *)&packet, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN, MAVLINK_MSG_ID_APNT_GPS_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_GPS_STATUS, (const char *)&packet, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_apnt_gps_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp_usec, float gps_lat, float gps_lon, float gps_alt, const uint8_t *prn, const int16_t *azimuth, const uint8_t *elevation, const uint8_t *snr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp_usec);
	_mav_put_float(buf, 8, gps_lat);
	_mav_put_float(buf, 12, gps_lon);
	_mav_put_float(buf, 16, gps_alt);
	_mav_put_int16_t_array(buf, 20, azimuth, 8);
	_mav_put_uint8_t_array(buf, 36, prn, 8);
	_mav_put_uint8_t_array(buf, 44, elevation, 8);
	_mav_put_uint8_t_array(buf, 52, snr, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_GPS_STATUS, buf, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN, MAVLINK_MSG_ID_APNT_GPS_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_GPS_STATUS, buf, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif
#else
	mavlink_apnt_gps_status_t *packet = (mavlink_apnt_gps_status_t *)msgbuf;
	packet->timestamp_usec = timestamp_usec;
	packet->gps_lat = gps_lat;
	packet->gps_lon = gps_lon;
	packet->gps_alt = gps_alt;
	mav_array_memcpy(packet->azimuth, azimuth, sizeof(int16_t)*8);
	mav_array_memcpy(packet->prn, prn, sizeof(uint8_t)*8);
	mav_array_memcpy(packet->elevation, elevation, sizeof(uint8_t)*8);
	mav_array_memcpy(packet->snr, snr, sizeof(uint8_t)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_GPS_STATUS, (const char *)packet, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN, MAVLINK_MSG_ID_APNT_GPS_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_APNT_GPS_STATUS, (const char *)packet, MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE APNT_GPS_STATUS UNPACKING


/**
 * @brief Get field timestamp_usec from apnt_gps_status message
 *
 * @return timestamp (microseconds since UNIX) from gps?
 */
static inline uint64_t mavlink_msg_apnt_gps_status_get_timestamp_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field gps_lat from apnt_gps_status message
 *
 * @return latitude from apnt gps
 */
static inline float mavlink_msg_apnt_gps_status_get_gps_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gps_lon from apnt_gps_status message
 *
 * @return longitude from apnt gps
 */
static inline float mavlink_msg_apnt_gps_status_get_gps_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gps_alt from apnt_gps_status message
 *
 * @return altitude from apnt gps
 */
static inline float mavlink_msg_apnt_gps_status_get_gps_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field prn from apnt_gps_status message
 *
 * @return array of prn code of up to 8 satellites in view
 */
static inline uint16_t mavlink_msg_apnt_gps_status_get_prn(const mavlink_message_t* msg, uint8_t *prn)
{
	return _MAV_RETURN_uint8_t_array(msg, prn, 8,  36);
}

/**
 * @brief Get field azimuth from apnt_gps_status message
 *
 * @return array of the azimuth of up to 8 satellites in view
 */
static inline uint16_t mavlink_msg_apnt_gps_status_get_azimuth(const mavlink_message_t* msg, int16_t *azimuth)
{
	return _MAV_RETURN_int16_t_array(msg, azimuth, 8,  20);
}

/**
 * @brief Get field elevation from apnt_gps_status message
 *
 * @return array of the elevation of up to 8 satellites in view
 */
static inline uint16_t mavlink_msg_apnt_gps_status_get_elevation(const mavlink_message_t* msg, uint8_t *elevation)
{
	return _MAV_RETURN_uint8_t_array(msg, elevation, 8,  44);
}

/**
 * @brief Get field snr from apnt_gps_status message
 *
 * @return array of signal to noise ration of up to 8 satellites in view
 */
static inline uint16_t mavlink_msg_apnt_gps_status_get_snr(const mavlink_message_t* msg, uint8_t *snr)
{
	return _MAV_RETURN_uint8_t_array(msg, snr, 8,  52);
}

/**
 * @brief Decode a apnt_gps_status message into a struct
 *
 * @param msg The message to decode
 * @param apnt_gps_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_apnt_gps_status_decode(const mavlink_message_t* msg, mavlink_apnt_gps_status_t* apnt_gps_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	apnt_gps_status->timestamp_usec = mavlink_msg_apnt_gps_status_get_timestamp_usec(msg);
	apnt_gps_status->gps_lat = mavlink_msg_apnt_gps_status_get_gps_lat(msg);
	apnt_gps_status->gps_lon = mavlink_msg_apnt_gps_status_get_gps_lon(msg);
	apnt_gps_status->gps_alt = mavlink_msg_apnt_gps_status_get_gps_alt(msg);
	mavlink_msg_apnt_gps_status_get_azimuth(msg, apnt_gps_status->azimuth);
	mavlink_msg_apnt_gps_status_get_prn(msg, apnt_gps_status->prn);
	mavlink_msg_apnt_gps_status_get_elevation(msg, apnt_gps_status->elevation);
	mavlink_msg_apnt_gps_status_get_snr(msg, apnt_gps_status->snr);
#else
	memcpy(apnt_gps_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_APNT_GPS_STATUS_LEN);
#endif
}
