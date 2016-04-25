/**
 * hunting_thread.cpp
 *
 * This is basically a revamp of the wifly thread.
 * Now contains everything within a class (which I guess could have been done with a namespace before)
 * Will communicate with the new RF detectors (through the arduino)
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

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

#include <vector>

#include "libs/udp/udp.h"
 #include "libs/bearing/bearing.h"

#include "common.h"
#include "commander.h"
#include "rfdetector_thread.h"

using std::vector;

RFDetector::RFDetector(struct MAVInfo* uavData, bool verbose) :
_jager(uavData),
_verbose(verbose),
_in_rotation(false),
_rotating(false),
_moving(false),
_send_next(false),
_curr_hunt_state(0),
_prev_hunt_state(0),
_dir_rssi(INT_MAX),
_omni_rssi(INT_MAX),
_meas_heading(0),
_bearing_cc(0),
_bearing_max(0),
_max_rssi(-100)
{
	/* "initialize" all the vectors */
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();
}

RFDetector::~RFDetector() {

}


void RFDetector::rotation_init() {

	printf("[HUNTING][STATE][ROT] rotation started\n");

	// set our logic to mark we are now running the rotation logic
	_in_rotation = true;

	// clear the vectors
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();
}


void RFDetector::rotation_completed() {
	printf("[HUNTING][STATE][ROT] ended rotation\n");

	// no longer in a rotation
	_in_rotation = false;

	if (_verbose) printf("[HUNTING] calculating end of rotation bearing...\n");

	/* get bearing and values */
	_bearing_cc = get_bearing_cc(_angles, _gains);		// do bearing calculation at this point
	_bearing_max = get_bearing_max(_angles, _gains);	// also do max bearing calculation
	_max_rssi = get_max_rssi(_gains);					// get what the max value was for the rssi

	if (_verbose) {
		printf("[HUNTING] calculated cc bearing: %f\n", _bearing_cc);
		printf("[HUNTING] calculated max bearing: %f\n", _bearing_max);
		printf("[HUNTING] max rssi value: %i\n", _max_rssi);
	}
}


void RFDetector::get_measurement() {
	if (_verbose) printf("[HUNTING] getting most recent measurement...\n");

	// defaults for all the values
	_dir_rssi = INT_MAX;
	_omni_rssi = INT_MAX;
	_meas_heading = _jager->vfr_hud.heading;

	// TODO: get the actual measurement (from the arduino link)
}


void RFDetector::check_hunt_state() {

	// check to see if the hunt state has changed (and if a command is required)
	if (_jager->tracking_status.hunt_mode_state != _curr_hunt_state) {

		printf("[HUNTING][STATE] State changed from %u to %u\n", _curr_hunt_state, _jager->tracking_status.hunt_mode_state);

		// update the prev hunt state to be the "current" state and update the current state to be the new current state
		_prev_hunt_state = _curr_hunt_state;
		_curr_hunt_state = _jager->tracking_status.hunt_mode_state;
		printf("[HUNTING][STATE] Prev State changed to: %u\n", _prev_hunt_state);

		// update state information
		update_state(_curr_hunt_state);

		// check to see if need to flag the next command to be sent
		// NOTE: want to send to the command at the end of this iteration to use the calculated data
		if (_curr_hunt_state == TRACKING_HUNT_STATE_WAIT) {
			printf("[HUNTING][CMD] flagging next command to be sent\n");
			_send_next = true;
		}	
	}
}


void RFDetector::update_state(const uint8_t &new_state) {
	
	switch (new_state) {
	case TRACKING_HUNT_STATE_WAIT:

		// mark that the rotation or moving has ended
		_rotating = false;
		_moving = false;
		break;

	case TRACKING_HUNT_STATE_ROTATE:

		// mark as now rotating
		_rotating = true;
		break;

	case TRACKING_HUNT_STATE_MOVE:

		// mark as now moving
		_moving = true;
		break;

	case TRACKING_HUNT_STATE_OFF:
		// finished = true;
		break;
	case TRACKING_HUNT_STATE_START:
		// starting = true;
		break;
	}
}

int RFDetector::get_max_rssi(const vector<double> rssi_values) {

	// set it to below what the RF detector can detect
	double max_rssi = -100;
	
	// loop through all values to get the max
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


int RFDetector::main_loop() {

	// some constants that all need to become parameters
	char *file_name = (char *) "wifly.csv";
	char *bearing_file_name = (char *) "bearing_calc_eor.csv";

	/* Open a file to write values to */
	/* Appending values */
	FILE *wifly_file = fopen(file_name, "a");
	if (wifly_file == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("[HUNTING] Error opening wifly file\n");
		return -1;
	}

	/* Open a file to write bearing calcs to */
	FILE *bearing_file = fopen(bearing_file_name, "a");
	if (bearing_file == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("[HUNTING] Error opening bearing output file\n");
		fclose(wifly_file);
		return -1;
	}


	/* open a UDP connection to send data down to the ground station */
	UDP* udp = new UDP();


	// TODO: move this into a class or something
	/* check if using specific commands */
	if (common::get_commands) {
		bool loaded = load_move_commands();
		if (!loaded) {
			printf("[HUNTING] Error loading move commands\n");
			fclose(wifly_file);
			fclose(bearing_file);
			return -1;
		}
	}
	

	struct timeval tv;
	unsigned long prev_loop_timestamp;

	// main loop that should be constantly taking measurements
	// until the main program is stopped
	while (common::RUNNING_FLAG) {

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

		printf("\n--------------------------------\n");
		printf("[HUNTING] Top of wifly loop\n");

		// check the hunt state from JAGER and adjust states accordingly
		check_hunt_state();

		//-----------------------------------------------//
		// make measurement, and get start and end angle (for during the measurement)
		//-----------------------------------------------//

		// TODO: potentially capture all measurements at the same time as 
		// making the rssi measurement (i.e. lat, lon, alt)
		// not sure what the variability is from here to later on

		get_measurement();

		//-----------------------------------------------//
		// Rotation specific calculations
		//-----------------------------------------------//

		/* check if we are in an official rotation */
		if (_rotating) {

			if (!_in_rotation) {
				rotation_init();
			}

			printf("[HUNTING][STATE][ROT] rotating\n");

			// add heading and rssi to the correct arrays (note will always get both dir and omni in this case)
			_angles.push_back((double) _meas_heading);
			_gains.push_back(_dir_rssi);
			_omni_gains.push_back(_omni_rssi);
		}


		/* catch the end of a rotation in order to do the cc gain measurement */
		if (!_rotating && _in_rotation) {
			rotation_completed();

			// save bearing cc to file (with important information)
			fprintf(bearing_file, "%llu,%i,%i,%f,%f,%f,%i\n", _jager->sys_time_us.time_unix_usec,
				_jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt, _bearing_cc, _bearing_max, _max_rssi);

			// send a mavlink message of the calculated bearing
			common::pixhawk->send_bearing_cc_message(_bearing_cc, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);

			// send the udp message (directly to ground)
			udp->send_bearing_message(TYPE_BEARING_CC, _bearing_cc, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);
		}


		//-----------------------------------------------//
		// save the measured data information to file
		//-----------------------------------------------//


		/* write the directional antenna information */
		fprintf(wifly_file, "%llu,%u,%i,%i,%i,%i,%i,%f,%i, %i\n",
				_jager->sys_time_us.time_unix_usec, _jager->custom_mode, _rotating, _meas_heading, _meas_heading,
				_jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt, _dir_rssi, _omni_rssi);

		// send a mavlink message with the current rssi
		common::pixhawk->send_rssi_message(_dir_rssi, _omni_rssi, _meas_heading, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);

		// send the udp message (directly to ground)
		// TODO: potentially only do this if we are in a rotation
		udp->send_rssi_message(_dir_rssi, _omni_rssi, _meas_heading, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);


		//-----------------------------------------------//
		// execute next command as needed
		//-----------------------------------------------//

		// if sending the next command has been flagged, send the next command, using the calculated data
		if (_send_next) {
			printf("[HUNTING][CMD] calling to send the next command...\n");
			send_next_command(_prev_hunt_state, _jager->tracking_status.hunt_mode_state, _bearing_max, _max_rssi);
			_send_next = false;
		}


	} // end of while running
	
	
	/* Be sure to close the output file and connection */
	fclose(wifly_file);
	fclose(bearing_file);

	delete &udp;

	return 0;
}



void *rfdetector_thread(void *param) {

	// just launch the hunter loop
	RFDetector* rfdetector_hunter = new RFDetector((struct MAVInfo *)param, common::verbose);
	return (void *) rfdetector_hunter->main_loop();
}