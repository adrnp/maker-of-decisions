// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <sys/time.h>
#include <limits.h>

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#
#ifdef __linux
#include <sys/ioctl.h>
#endif

//#include "serialwifly.h" 	// this is from Louis' wifly code, which will be a library
#include "serial_lib/wifly_serial.h" // include the new class for handling a wifly
#include "common.h"
#include "wifly_thread.h"
// #include "test.h" 			// for bearing calculation
#include "commander.h"
#include "bearing_lib/bearing.h"		// this should be all the functions required for bearing calculations

using std::string;
using std::vector;
using namespace std;

/* this is to help with some of the rotation logic */
bool in_rotation = false;

/* keep track of the previous hunt state, as hunt state is sent periodically, not just on updates */
uint8_t prev_hunt_state = 10;

/* whether or not we are currently rotating */
bool rotating = false;

/* whether or not we are currently moving */
bool moving = false;

/* microsecond timestamp of the previous loop iteration */
unsigned long prev_loop_timestamp = 0;


void update_state(uint8_t &new_state) {

	switch (new_state) {
	case TRACKING_HUNT_STATE_ROTATE:
		rotating = true;
		break;
	case TRACKING_HUNT_STATE_MOVE:
		moving = true;
		break;
	case TRACKING_HUNT_STATE_OFF:
		// finished = true;
		break;
	case TRACKING_HUNT_STATE_START:
		// starting = true;
		break;
	}
}


void *wifly_thread(void *param) {
	
	// retrieve the MAVInfo struct that is sent to this function as a parameter
	struct MAVInfo *uavData = (struct MAVInfo *)param;
	
	// some constants that all need to become parameters
	int num_samples = 1;
	char *ssid = (char *) "JAMMER01";
	char *file_name = (char *) "wifly.csv";
	char *file_name2 = (char *) "wifly2.csv";
	char *bearing_file_name = (char *) "bearing_calc_cc.csv";
	char *bearing_mle_file_name = (char *) "bearing_calc_mle.csv";
	// char *port = (char *) "/dev/ttyUSB0";

	// connect to the first wifly
	WiflySerial wifly1(false, wifly_port1);
	if (wifly1.fd < 0) {
		printf("Error opening wifly connection\n");
		return NULL;
	}

	// connect to the second wifly
	WiflySerial wifly2(false);
	if (dual_wifly) {
		wifly2.open_serial(wifly_port2);
		if (wifly2.fd < 0) {
			printf("Error opening wifly connection\n");
			return NULL;
		}
	}

	
	/* Go into command mode */
	wifly1.enter_commandmode();
	if (dual_wifly) {
		wifly2.enter_commandmode();
	}


	/* Open a file to write values to */
	/* Appending values */
	FILE *wifly_file = fopen(file_name, "a");
	if (wifly_file == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("Error opening wifly output file\n");
		return NULL;
	}


	FILE *wifly_file2 = NULL;
	if (dual_wifly) {
		wifly_file2 = fopen(file_name2, "a");
		if (wifly_file2 == NULL) {
			// TODO: figure out what we do want to return when there is an error
			printf("Error opening wifly2 output file\n");
			return NULL;
		}
	}
	

	/* Open a file to write bearing calcs to */
	FILE *bearing_file = fopen(bearing_file_name, "a");
	if (bearing_file == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("Error opening bearing output file\n");
		return NULL;
	}

	FILE *bearing_file_mle = NULL;
	if (dual_wifly) {
		bearing_file_mle = fopen(bearing_mle_file_name, "a");
		if (bearing_file_mle == NULL) {
			printf("Error opening bearing mle output file\n");
			return NULL;
		}
	}


	if (get_commands) {
		bool loaded = load_move_commands();
		if (!loaded) {
			printf("Error loading move commands\n");
			return NULL;
		}
	}
	
	vector<double> angles;
	vector<double> gains;
	vector<double> omni_gains;
	vector<int> norm_gains;

	struct timeval tv;



	// main loop that should be constantly taking measurements
	// until the main program is stopped
	while (RUNNING_FLAG) {

		// only want to execute this at most ever 30 ms
		// basically does a dynamic sleep in the sense that if there is a lot of processing time for doing the bearing
		// calculation, we will only pause as long as required so measurements are really made every 30 ms
		// (unless of course bearing calculations take too long)
		gettimeofday(&tv, NULL);
		unsigned long current_loop_time = 1000000 * tv.tv_sec + tv.tv_usec;
		if (prev_loop_timestamp != 0 && (current_loop_time - prev_loop_timestamp) < 30000) {
			continue;
		}
		prev_loop_timestamp = current_loop_time;

		// handle hunt state changes required (sending of commands)
		if (uavData->tracking_status.hunt_mode_state != prev_hunt_state) {
			
			printf("State changed from %u to %u\n", prev_hunt_state, uavData->tracking_status.hunt_mode_state);

			if (uavData->tracking_status.hunt_mode_state == TRACKING_HUNT_STATE_WAIT) {
				send_next_command(prev_hunt_state, uavData->tracking_status.hunt_mode_state);
				
				// TODO: maybe want to update the state immediately here...
			} else {
				update_state(uavData->tracking_status.hunt_mode_state);
			}
			
			
			// update the prev hunt state to be this new state
			prev_hunt_state = uavData->tracking_status.hunt_mode_state;
			printf("Prev State changed to: %u\n", prev_hunt_state);
		}
	
		// make measurements (depending on the current state)
		int dir_rssi = INT_MAX;
		int omni_rssi = INT_MAX;

		/* check if we are in an official rotation */
		if (rotating) {
			if (!in_rotation) {
				printf("rotation started\n");
				// set our logic to mark we are now running the rotation logic
				in_rotation = true;

				// clear the vectors
				angles.clear();
				gains.clear();
				omni_gains.clear();
				norm_gains.clear();
			}

			printf("rotating\n");
			angles.push_back((double) uavData->vfr_hud.heading);
			dir_rssi = wifly1.scanrssi(ssid);
			gains.push_back(dir_rssi);

			if (dual_wifly) {
				omni_rssi = wifly2.scanrssi(ssid);
				omni_gains.push_back(omni_rssi);

				// calculate the normalized gains
				norm_gains.push_back(gains2normgain(dir_rssi, omni_rssi));

				// do constant calculation of bearing
				printf("calculating bearing mle\n");
				double curr_bearing_est = get_bearing_mle(angles, norm_gains);
				fprintf(bearing_file_mle, "%llu, %i,%i,%f,%f\n", uavData->sys_time_us.time_unix_usec, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, curr_bearing_est);

				// TODO: send mle bearing message here!

			}
		}

		/* catch the end of a rotation in order to do the cc gain measurement */
		if (!rotating && in_rotation) {
			printf("ended rotation\n");
			in_rotation = false;

			printf("calculating cc bearing\n");
			// do bearing calculation at this point
			// double bearing = get_bearing_cc(angles, gains);
			double bearing = 32.0;

			// write the lat, lon, alt and bearing to file
			fprintf(bearing_file, "%llu, %i,%i,%f,%f\n", uavData->sys_time_us.time_unix_usec, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, bearing);

			// send a mavlink message of the calculated bearing
			send_bearing_message(bearing, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

			// tell pixhawk we are finished with the rotation
			// send_finish_command();
		}

		/* no need to make another measurement to write to the file if we already made one */
		if (rotating) {
			printf("writing directly to file\n");
			/* write the directional measurement information */
			fprintf(wifly_file, "%llu,%u,%i,%i,%i,%f,%i\n",
				uavData->sys_time_us.time_unix_usec, uavData->custom_mode, uavData->vfr_hud.heading,
				uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, dir_rssi);

			/* write the omni measurement as needed */
			if (dual_wifly) {
				fprintf(wifly_file2, "%llu,%u,%i,%i,%i,%f,%i\n",
					uavData->sys_time_us.time_unix_usec, uavData->custom_mode, uavData->vfr_hud.heading,
					uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, omni_rssi);
			}
		
		} else { /* need to make a measurement to save to the file */

			printf("making measurement for writing to file\n");

			/* Scan values to this file */
			/* Add degree at which you measure first */
			fprintf(wifly_file, "%llu,%u,%i,%i,%i,%f,",
				uavData->sys_time_us.time_unix_usec, uavData->custom_mode, uavData->vfr_hud.heading,
				uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

			dir_rssi = wifly1.scanrssi_f(ssid, wifly_file, num_samples);
			cout << uavData->vfr_hud.heading << ": wifly1: " << dir_rssi << "\n";

			if (dual_wifly) {

				fprintf(wifly_file2, "%llu,%u,%i,%i,%i,%f,",
					uavData->sys_time_us.time_unix_usec, uavData->custom_mode, uavData->vfr_hud.heading,
					uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

				omni_rssi = wifly2.scanrssi_f(ssid, wifly_file2, num_samples);
				cout << uavData->vfr_hud.heading << ": wifly2: " << omni_rssi << "\n";
			}

		}

		// send a mavlink message with the current rssi
		send_rssi_message(dir_rssi, uavData->vfr_hud.heading, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

		// TODO: send mavlink message of calculated rssi...

		// send a message with a rotate command for testing purposes at the moment
		// sendRotateCommand(-1.0);
		
		/* sleep for some time before making another measurement (30 ms for now) */
		// usleep(30000);
		
	}
	
	
	/* Be sure to close the output file and connection */
	fclose(wifly_file);
	fclose(bearing_file);
	wifly1.end_serial();

	if (dual_wifly) {
		fclose(wifly_file2);
		fclose(bearing_file_mle);
		wifly2.end_serial();
	}

	return NULL;
}
