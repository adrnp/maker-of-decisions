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

/* keep track of the current previous hunt state, as hunt state is sent periodically, not just on updates */
uint8_t current_hunt_state = 0;
uint8_t prev_hunt_state = 0;

/* whether or not we are currently rotating */
bool rotating = false;

/* whether or not we are currently moving */
bool moving = false;

/* microsecond timestamp of the previous loop iteration */
unsigned long prev_loop_timestamp = 0;

unsigned long prev_omni_update_timestamp = 0;

void update_state(uint8_t &new_state) {

	switch (new_state) {
	case TRACKING_HUNT_STATE_WAIT:

		// mark that the rotation or moving has ended
		rotating = false;
		moving = false;
		break;

	case TRACKING_HUNT_STATE_ROTATE:

		// mark as now rotating
		rotating = true;
		break;

	case TRACKING_HUNT_STATE_MOVE:

		// mark as now moving
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


int get_max_rssi(vector<double> rssi_values) {

	double max_rssi = -100;
	
	int len = rssi_values.size();
	for (int i = 0; i < len; i++) {
		
		// ignore any invalid measurement
		if (rssi_values[i] == INT_MAX) {
			continue;
		}

		// update max value
		if (rssi_values[i] > max_rssi) {
			max_rssi = rssi_values[i];
		}
	}

	// return the true max value
	return (int) max_rssi;
}


void *wifly_thread(void *param) {
	
	// retrieve the MAVInfo struct that is sent to this function as a parameter
	struct MAVInfo *uavData = (struct MAVInfo *)param;
	
	// some constants that all need to become parameters
	char *ssid = (char *) "JAMMER01"; // "ADL"; // "JAMMER01";
	char *file_name = (char *) "wifly.csv";
	char *bearing_file_name = (char *) "bearing_calc_eor.csv";
	char *bearing_mle_file_name = (char *) "bearing_calc_mle.csv";

	// connect to the first wifly
	WiflySerial* wifly1 = new WiflySerial(verbose, wifly_port1);
	if (wifly1->fd < 0) {
		printf("Error opening wifly connection\n");
		return NULL;
	}

	/* Go into command mode */
	wifly1->enter_commandmode();

	/* Open a file to write values to */
	/* Appending values */
	FILE *wifly_file = fopen(file_name, "a");
	if (wifly_file == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("Error opening wifly output file\n");
		return NULL;
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

	double bearing_cc;
	double bearing_max;
	int max_rssi;

	bool send_next = false;

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
		if (uavData->tracking_status.hunt_mode_state != current_hunt_state) {
			
			printf("State changed from %u to %u\n", current_hunt_state, uavData->tracking_status.hunt_mode_state);

			// update the prev hunt state to be the "current" state and update the current state to be the new current state
			prev_hunt_state = current_hunt_state;
			current_hunt_state = uavData->tracking_status.hunt_mode_state;
			printf("Prev State changed to: %u\n", prev_hunt_state);

			// update state information
			update_state(current_hunt_state);

			// check to see if need to flag the next command to be sent
			// NOTE: want to send to the command at the end of this iteration to use the calculated data
			if (current_hunt_state == TRACKING_HUNT_STATE_WAIT) {
				printf("flagging next command to be sent\n");
				send_next = true;

				// TODO: maybe want to update the state immediately here...
				// send_next_command(prev_hunt_state, uavData->tracking_status.hunt_mode_state, bearing_cc, max_rssi);
			}

			
		}

		//-----------------------------------------------//
		// make measurement, and get start and end angle (for during the measurement)
		//-----------------------------------------------//

		// TODO: potentially capture all measurements at the same time as 
		// making the rssi measurement (i.e. lat, lon, alt)
		// not sure what the variability is from here to later on

		int dir_rssi = INT_MAX;

		if (verbose) printf("scanning wifly 1...\n");
		int16_t heading_dir_pre = uavData->vfr_hud.heading;
		dir_rssi = wifly1->scanrssi(ssid);
		
		if (verbose) printf("dir rssi recevied: %i\n", dir_rssi);
		
		int16_t heading_dir_post = uavData->vfr_hud.heading;

		//-----------------------------------------------//
		// Rotation specific calculations
		//-----------------------------------------------//

		/* check if we are in an official rotation */
		if (rotating) {
			if (!in_rotation) {
				if (verbose) printf("rotation started\n");
				// set our logic to mark we are now running the rotation logic
				in_rotation = true;

				// clear the vectors
				angles.clear();
				gains.clear();
				omni_gains.clear();
				norm_gains.clear();
			}

			if (verbose) printf("rotating\n");

			// add heading and rssi to the correct arrays
			angles.push_back((double) heading_dir_pre);
			gains.push_back(dir_rssi);

			// if using both wiflies, need to see if there was an omni update
			// may have a problem with omni value not matching the same location as the dir measurement
			if (dual_wifly && omni_update_timestamp != prev_omni_update_timestamp) {
				
				// add omni rssi to the correct array
				omni_gains.push_back(omni_rssi);

				// calculate the normalized gains
				norm_gains.push_back(gains2normgain(dir_rssi, omni_rssi));

				// do constant calculation of bearing
				if (verbose) printf("calculating bearing mle\n");
				double curr_bearing_est = get_bearing_mle(angles, norm_gains);

				// save the calculated mle bearing
				fprintf(bearing_file_mle, "%llu,%i,%i,%f,%f\n", uavData->sys_time_us.time_unix_usec,
					uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, curr_bearing_est);

				// send a mavlink message of the calculated mle bearing
				send_bearing_mle_message(curr_bearing_est, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

			}
		}

		/* catch the end of a rotation in order to do the cc gain measurement */
		if (!rotating && in_rotation) {
			if (verbose) printf("ended rotation\n");
			in_rotation = false;

			if (verbose) printf("calculating end of rotation bearing...\n");

			// do bearing calculation at this point
			bearing_cc = get_bearing_cc(angles, gains);
			if (verbose) printf("calculated cc bearing: %f\n", bearing_cc);

			// also do max bearing calculation
			bearing_max = get_bearing_max(angles, gains);
			if (verbose) printf("calculated max bearing: %f\n", bearing_max);

			// get what the max value was for the rssi
			max_rssi = get_max_rssi(gains);
			if (verbose) printf("max rssi value: %i\n", max_rssi);

			// save bearing cc to file (with important information)
			fprintf(bearing_file, "%llu,%i,%i,%f,%f,%f,%i\n", uavData->sys_time_us.time_unix_usec,
				uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, bearing_cc, bearing_max, max_rssi);

			// send a mavlink message of the calculated bearing
			send_bearing_cc_message(bearing_cc, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

			// send a mavlink message of the max bearing over the mle message for now
			// send_bearing_mle_message(bearing_max, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

			// tell pixhawk we are finished with the rotation
			// send_finish_command();
		}

		//-----------------------------------------------//
		// save the measured data information to file
		//-----------------------------------------------//


		/* write the directional atenna information */
		printf("writing dir rssi to file: %i\n", dir_rssi);
		fprintf(wifly_file, "%llu,%u,%i,%i,%i,%i,%i,%f,%i\n",
				uavData->sys_time_us.time_unix_usec, uavData->custom_mode, rotating, heading_dir_pre, heading_dir_post,
				uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, dir_rssi);

		// send a mavlink message with the current rssi
		send_rssi_message(dir_rssi, omni_rssi, heading_dir_pre, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);



		//-----------------------------------------------//
		// execute next command as needed
		//-----------------------------------------------//

		// if sending the next command has been flagged, send the next command, using the calculated data
		if (send_next) {
			printf("calling to send the next command...\n");
			send_next_command(prev_hunt_state, uavData->tracking_status.hunt_mode_state, bearing_max, max_rssi);
			send_next = false;
		}



	} // end of while running
	
	
	/* Be sure to close the output file and connection */
	fclose(wifly_file);
	fclose(bearing_file);
	wifly1->end_serial();

	if (dual_wifly) {
		fclose(bearing_file_mle);
	}

	return NULL;
}
