/** @file
 *	@brief MAVLink comm protocol testsuite generated from jager.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef JAGER_TESTSUITE_H
#define JAGER_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_jager(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_jager(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_apnt_gps_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_apnt_gps_status_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	}129.0,
	}{ 18275, 18276, 18277, 18278, 18279, 18280, 18281, 18282 },
	}{ 113, 114, 115, 116, 117, 118, 119, 120 },
	}{ 137, 138, 139, 140, 141, 142, 143, 144 },
	}{ 161, 162, 163, 164, 165, 166, 167, 168 },
	};
	mavlink_apnt_gps_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp_usec = packet_in.timestamp_usec;
        	packet1.gps_lat = packet_in.gps_lat;
        	packet1.gps_lon = packet_in.gps_lon;
        	packet1.gps_alt = packet_in.gps_alt;
        
        	mav_array_memcpy(packet1.azimuth, packet_in.azimuth, sizeof(int16_t)*8);
        	mav_array_memcpy(packet1.prn, packet_in.prn, sizeof(uint8_t)*8);
        	mav_array_memcpy(packet1.elevation, packet_in.elevation, sizeof(uint8_t)*8);
        	mav_array_memcpy(packet1.snr, packet_in.snr, sizeof(uint8_t)*8);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_gps_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_apnt_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_gps_status_pack(system_id, component_id, &msg , packet1.timestamp_usec , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.prn , packet1.azimuth , packet1.elevation , packet1.snr );
	mavlink_msg_apnt_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_gps_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_usec , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.prn , packet1.azimuth , packet1.elevation , packet1.snr );
	mavlink_msg_apnt_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_apnt_gps_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_gps_status_send(MAVLINK_COMM_1 , packet1.timestamp_usec , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.prn , packet1.azimuth , packet1.elevation , packet1.snr );
	mavlink_msg_apnt_gps_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_apnt_site_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_apnt_site_status_t packet_in = {
		93372036854775807ULL,
	}{ 963497880, 963497881, 963497882, 963497883 },
	}{ 185.0, 186.0, 187.0, 188.0 },
	}{ 297.0, 298.0, 299.0, 300.0 },
	}{ 20147, 20148, 20149, 20150 },
	};
	mavlink_apnt_site_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp_usec = packet_in.timestamp_usec;
        
        	mav_array_memcpy(packet1.site_id, packet_in.site_id, sizeof(uint32_t)*4);
        	mav_array_memcpy(packet1.site_lat, packet_in.site_lat, sizeof(float)*4);
        	mav_array_memcpy(packet1.site_lon, packet_in.site_lon, sizeof(float)*4);
        	mav_array_memcpy(packet1.site_signal, packet_in.site_signal, sizeof(uint16_t)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_site_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_apnt_site_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_site_status_pack(system_id, component_id, &msg , packet1.timestamp_usec , packet1.site_id , packet1.site_lat , packet1.site_lon , packet1.site_signal );
	mavlink_msg_apnt_site_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_site_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_usec , packet1.site_id , packet1.site_lat , packet1.site_lon , packet1.site_signal );
	mavlink_msg_apnt_site_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_apnt_site_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_site_status_send(MAVLINK_COMM_1 , packet1.timestamp_usec , packet1.site_id , packet1.site_lat , packet1.site_lon , packet1.site_signal );
	mavlink_msg_apnt_site_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_apnt_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_apnt_position_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	};
	mavlink_apnt_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp_usec = packet_in.timestamp_usec;
        	packet1.apnt_lat = packet_in.apnt_lat;
        	packet1.apnt_lon = packet_in.apnt_lon;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_position_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_apnt_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_position_pack(system_id, component_id, &msg , packet1.timestamp_usec , packet1.apnt_lat , packet1.apnt_lon );
	mavlink_msg_apnt_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_usec , packet1.apnt_lat , packet1.apnt_lon );
	mavlink_msg_apnt_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_apnt_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_apnt_position_send(MAVLINK_COMM_1 , packet1.timestamp_usec , packet1.apnt_lat , packet1.apnt_lon );
	mavlink_msg_apnt_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_tracking_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_tracking_status_t packet_in = {
		93372036854775807ULL,
	}29,
	}96,
	};
	mavlink_tracking_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp_usec = packet_in.timestamp_usec;
        	packet1.computer_status = packet_in.computer_status;
        	packet1.hunt_mode_state = packet_in.hunt_mode_state;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tracking_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_tracking_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tracking_status_pack(system_id, component_id, &msg , packet1.timestamp_usec , packet1.computer_status , packet1.hunt_mode_state );
	mavlink_msg_tracking_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tracking_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_usec , packet1.computer_status , packet1.hunt_mode_state );
	mavlink_msg_tracking_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_tracking_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tracking_status_send(MAVLINK_COMM_1 , packet1.timestamp_usec , packet1.computer_status , packet1.hunt_mode_state );
	mavlink_msg_tracking_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_tracking_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_tracking_cmd_t packet_in = {
		93372036854775807ULL,
	}179.0,
	}235.0,
	}185.0,
	}213.0,
	}18899,
	}235,
	};
	mavlink_tracking_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp_usec = packet_in.timestamp_usec;
        	packet1.north = packet_in.north;
        	packet1.east = packet_in.east;
        	packet1.yaw_angle = packet_in.yaw_angle;
        	packet1.altitude = packet_in.altitude;
        	packet1.cmd_id = packet_in.cmd_id;
        	packet1.cmd_type = packet_in.cmd_type;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tracking_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_tracking_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tracking_cmd_pack(system_id, component_id, &msg , packet1.timestamp_usec , packet1.cmd_id , packet1.cmd_type , packet1.north , packet1.east , packet1.yaw_angle , packet1.altitude );
	mavlink_msg_tracking_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tracking_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp_usec , packet1.cmd_id , packet1.cmd_type , packet1.north , packet1.east , packet1.yaw_angle , packet1.altitude );
	mavlink_msg_tracking_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_tracking_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tracking_cmd_send(MAVLINK_COMM_1 , packet1.timestamp_usec , packet1.cmd_id , packet1.cmd_type , packet1.north , packet1.east , packet1.yaw_angle , packet1.altitude );
	mavlink_msg_tracking_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_hunt_mission_current(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_hunt_mission_current_t packet_in = {
		17235,
	};
	mavlink_hunt_mission_current_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.current_cmd_id = packet_in.current_cmd_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hunt_mission_current_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_hunt_mission_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hunt_mission_current_pack(system_id, component_id, &msg , packet1.current_cmd_id );
	mavlink_msg_hunt_mission_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hunt_mission_current_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.current_cmd_id );
	mavlink_msg_hunt_mission_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_hunt_mission_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hunt_mission_current_send(MAVLINK_COMM_1 , packet1.current_cmd_id );
	mavlink_msg_hunt_mission_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_hunt_mission_reached(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_hunt_mission_reached_t packet_in = {
		17235,
	};
	mavlink_hunt_mission_reached_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.reached_cmd_id = packet_in.reached_cmd_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hunt_mission_reached_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_hunt_mission_reached_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hunt_mission_reached_pack(system_id, component_id, &msg , packet1.reached_cmd_id );
	mavlink_msg_hunt_mission_reached_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hunt_mission_reached_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.reached_cmd_id );
	mavlink_msg_hunt_mission_reached_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_hunt_mission_reached_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hunt_mission_reached_send(MAVLINK_COMM_1 , packet1.reached_cmd_id );
	mavlink_msg_hunt_mission_reached_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_jager(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_apnt_gps_status(system_id, component_id, last_msg);
	mavlink_test_apnt_site_status(system_id, component_id, last_msg);
	mavlink_test_apnt_position(system_id, component_id, last_msg);
	mavlink_test_tracking_status(system_id, component_id, last_msg);
	mavlink_test_tracking_cmd(system_id, component_id, last_msg);
	mavlink_test_hunt_mission_current(system_id, component_id, last_msg);
	mavlink_test_hunt_mission_reached(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // JAGER_TESTSUITE_H
