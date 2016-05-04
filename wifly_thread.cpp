/**
 * @file wifly_thread.cpp
 *
 * Implements the wifly thread and WiflyHunter class.
 * reads from the wifly modules and triggers commands to be sent accordingly.
 * 
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

#include "libs/serial/wifly_serial.h" 	// include the new class for handling a wifly
#include "libs/bearing/bearing.h"		// this should be all the functions required for bearing calculations
#include "libs/udp/udp.h"				// udp communication
#include "common.h"
#include "commander.h"
#include "wifly_thread.h"


using std::string;
using std::vector;
using namespace std;



WiflyHunter::WiflyHunter(struct MAVInfo* uavData, bool verbose) :
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
_heading_dir_pre(0),
_heading_dir_post(0),
_heading_omni_post(0),
_bearing_cc(0),
_bearing_max(0),
_bearing_max3(0),
_max_rssi(-100)
{
	/* "initialize" all the vectors */
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();
}

WiflyHunter::~WiflyHunter() {

}


void WiflyHunter::rotation_init() {

	LOG_STATUS("[WIFLY][STATE][ROT] rotation started");

	// set our logic to mark we are now running the rotation logic
	_in_rotation = true;

	// clear the vectors
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();
}


void WiflyHunter::rotation_completed() {
	LOG_STATUS("[WIFLY][STATE][ROT] ended rotation");

	// no longer in a rotation
	_in_rotation = false;

	LOG_DEBUG("[WIFLY] calculating end of rotation bearing...");

	/* get bearing and values */
	_bearing_cc = get_bearing_cc(_angles, _gains);		// do bearing calculation at this point
	_bearing_max = get_bearing_max(_angles, _gains);	// also do max bearing calculation
	_bearing_max3 = get_bearing_max3(_angles, _gains);	// do meax3 bearing calculation
	_max_rssi = get_max_rssi(_gains);					// get what the max value was for the rssi

	if (_verbose) {
		LOG_STATUS("[WIFLY] calculated cc bearing: %f", _bearing_cc);
		LOG_STATUS("[WIFLY] calculated max bearing: %f", _bearing_max);
		LOG_STATUS("[WIFLY] max rssi value: %i", _max_rssi);
	}
}


void WiflyHunter::check_hunt_state() {

	// check to see if the hunt state has changed (and if a command is required)
	if (_jager->tracking_status.hunt_mode_state != _curr_hunt_state) {

		LOG_STATUS("[WIFLY][STATE] State changed from %u to %u", _curr_hunt_state, _jager->tracking_status.hunt_mode_state);

		// update the prev hunt state to be the "current" state and update the current state to be the new current state
		_prev_hunt_state = _curr_hunt_state;
		_curr_hunt_state = _jager->tracking_status.hunt_mode_state;
		LOG_STATUS("[WIFLY][STATE] Prev State changed to: %u", _prev_hunt_state);

		// update state information
		update_state(_curr_hunt_state);

		// check to see if need to flag the next command to be sent
		// NOTE: want to send to the command at the end of this iteration to use the calculated data
		if (_curr_hunt_state == TRACKING_HUNT_STATE_WAIT) {
			LOG_STATUS("[WIFLY][CMD] flagging next command to be sent");
			_send_next = true;
		}	
	}
}


void WiflyHunter::update_state(const uint8_t &new_state) {
	
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

int WiflyHunter::get_max_rssi(const vector<double> rssi_values) {

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


int WiflyHunter::main_loop() {

	// some constants that all need to become parameters
	char *ssid = (char *) "JAMMER01"; // "ADL"; // "JAMMER01";		// the SSID that the wifly is looking for

	string rssi_logfile_name = common::logfile_dir + "rssi.csv";					// the logfile for the rssi values
	string bearing_logfile_name = common::logfile_dir + "bearing_calc_eor.csv";		// the logfile for the end of rotation bearing calculations
	string bearing_mle_logfile_name = common::logfile_dir + "bearing_calc_mle.csv";	// the logfile for the mle bearing estimates

	// connect to the first wifly
	WiflySerial* wifly1 = new WiflySerial(_verbose, common::sensor_port);
	if (wifly1->fd < 0) {
		LOG_STATUS("[WIFLY] Error opening wifly connection");
		return -1;
	}

	LOG_STATUS("[WIFLY] wifly 1 fd is %d", wifly1->fd);

	/* Go into command mode */
	wifly1->enter_commandmode();

	/* Open a file to write values to */
	/* Appending values */
	FILE *rssi_logfile = fopen(rssi_logfile_name.c_str(), "a");
	if (rssi_logfile == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		LOG_STATUS("[WIFLY] Error opening wifly file");
		return -1;
	}

	/* Open a file to write bearing calcs to */
	FILE *bearing_logfile = fopen(bearing_logfile_name.c_str(), "a");
	if (bearing_logfile == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		LOG_STATUS("[WIFLY] Error opening bearing output file");
		fclose(rssi_logfile);
		return -1;
	}

	FILE *bearing_logfile_mle = NULL;
	WiflySerial* wifly2 = NULL;
	if (common::dual_wifly) {
		bearing_logfile_mle = fopen(bearing_mle_logfile_name.c_str(), "a");
		if (bearing_logfile_mle == NULL) {
			LOG_STATUS("[WIFLY] Error opening bearing mle output file");
			fclose(rssi_logfile);
			fclose(bearing_logfile);
			return -1;
		}

		wifly2 = new WiflySerial(_verbose, common::omni_wifly_port);
		if (wifly2->fd < 0) {
			LOG_STATUS("[WIFLY] Error opening wifly 2 connection");
			fclose(rssi_logfile);
			fclose(bearing_logfile);
			fclose(bearing_logfile_mle);
			return -1;
		}
		LOG_STATUS("[WIFLY] wifly 2 fd is %d", wifly2->fd);

		/* enter command mode */
		wifly2->enter_commandmode();
	}

	/* open a UDP connection to send data down to the ground station */
	UDP* udp = new UDP();

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

		LOG_STATUS("\n--------------------------------");
		LOG_STATUS("[WIFLY] Top of wifly loop");

		// check the hunt state from JAGER and adjust states accordingly
		check_hunt_state();

		//-----------------------------------------------//
		// make measurement, and get start and end angle (for during the measurement)
		//-----------------------------------------------//

		// TODO: potentially capture all measurements at the same time as 
		// making the rssi measurement (i.e. lat, lon, alt)
		// not sure what the variability is from here to later on

		LOG_STATUS("[WIFLY] making measurement...");
		_dir_rssi = INT_MAX;
		_omni_rssi = INT_MAX;

		LOG_DEBUG("[WIFLY] scanning wifly 1...");
		_heading_dir_pre = _jager->vfr_hud.heading;
		_dir_rssi = wifly1->scanrssi(ssid);
		_heading_dir_post = _jager->vfr_hud.heading;
		LOG_DEBUG("[WIFLY] dir rssi recevied: %i", _dir_rssi);
		
		if (common::dual_wifly) {
			LOG_DEBUG("[WIFLY] scanning wifly 2...");
			_omni_rssi = wifly2->scanrssi(ssid);
			_heading_omni_post = _jager->vfr_hud.heading;
			LOG_DEBUG("[WIFLY] omni rssi recevied: %i", _omni_rssi);
		}

		//-----------------------------------------------//
		// Rotation specific calculations
		//-----------------------------------------------//

		/* check if we are in an official rotation */
		if (_rotating) {

			if (!_in_rotation) {
				rotation_init();
				common::planner->reset_observations();
			}

			LOG_STATUS("[WIFLY][STATE][ROT] rotating");

			// add heading and rssi to the correct arrays (note will always get both dir and omni in this case)
			_angles.push_back((double) _heading_dir_pre);
			_gains.push_back(_dir_rssi);
			
			// if using both wiflies, need to see if there was an omni update
			// may have a problem with omni value not matching the same location as the dir measurement
			if (common::dual_wifly) {
				
				// add omni rssi to the correct array
				_omni_gains.push_back(_omni_rssi);

				// calculate the normalized gains
				_norm_gains.push_back(gains2normgain(_dir_rssi, _omni_rssi));

				// do constant calculation of bearing
				LOG_DEBUG("[WIFLY] calculating bearing mle");
				double curr_bearing_est = get_bearing_mle(_angles, _norm_gains);

				// save the calculated mle bearing
				fprintf(bearing_logfile_mle, "%llu,%i,%i,%f,%f\n", _jager->sys_time_us.time_unix_usec,
					_jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt, curr_bearing_est);

				/* send data */
				common::pixhawk->send_bearing_mle_message(curr_bearing_est, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);		// send a mavlink message of the calculated mle bearing
				udp->send_bearing_message(TYPE_BEARING_MLE, curr_bearing_est, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);		// send the udp message (directly to ground)
			}

			// update the planner's most recent observations
			common::planner->update_observation(_heading_dir_pre, _dir_rssi, _omni_rssi);
		}


		/* catch the end of a rotation in order to do the cc gain measurement */
		if (!_rotating && _in_rotation) {
			rotation_completed();

			// update the planner's full observations
			common::planner->update_observations(_angles, _gains, _omni_gains, _norm_gains, _bearing_cc, _bearing_max, _bearing_max3);

			// save bearing cc to file (with important information)
			fprintf(bearing_logfile, "%llu,%i,%i,%f,%f,%f,%i\n", _jager->sys_time_us.time_unix_usec,
				_jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt, _bearing_cc, _bearing_max, _max_rssi);

			/* send data */
			common::pixhawk->send_bearing_cc_message(_bearing_cc, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);		// send a mavlink message of the calculated bearing
			udp->send_bearing_message(TYPE_BEARING_CC, _bearing_cc, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);	// send the udp message (directly to ground)
		}


		//-----------------------------------------------//
		// save the measured data information to file
		//-----------------------------------------------//


		/* write the directional antenna information */
		fprintf(rssi_logfile, "%llu,%u,%i,%i,%i,%i,%i,%f,%i, %i\n",
				_jager->sys_time_us.time_unix_usec, _jager->custom_mode, _rotating, _heading_dir_pre, _heading_dir_post,
				_jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt, _dir_rssi, _omni_rssi);

		/* send data */
		common::pixhawk->send_rssi_message(_dir_rssi, _omni_rssi, _heading_dir_pre, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);	// send a mavlink message with the current rssi
		udp->send_rssi_message(_dir_rssi, _omni_rssi, _heading_dir_pre, _jager->gps_position.lat, _jager->gps_position.lon, _jager->vfr_hud.alt);				// send the udp message (directly to ground)


		//-----------------------------------------------//
		// execute next command as needed
		//-----------------------------------------------//

		// if sending the next command has been flagged, send the next command, using the calculated data
		if (_send_next) {
			LOG_STATUS("[WIFLY][CMD] calling to send the next command");
			send_next_command(_prev_hunt_state, _jager->tracking_status.hunt_mode_state);
			_send_next = false;
		}


	} // end of while running
	
	
	/* Be sure to close the output file and connection */
	fclose(rssi_logfile);
	fclose(bearing_logfile);
	wifly1->end_serial();

	delete &udp;


	if (common::dual_wifly) {
		fclose(bearing_logfile_mle);
		wifly2->end_serial();
	}

	return 0;
}




void *wifly_thread(void *param) {
	// just launch the wifly loop
	WiflyHunter* wifly_hunter = new WiflyHunter((struct MAVInfo *)param, common::verbose);
	return (void *) wifly_hunter->main_loop();
}
