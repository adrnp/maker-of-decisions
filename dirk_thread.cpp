/**
 * @file dirk_thread.cpp
 *
 * The implimentation of the thread handling the beam steering antenna.
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

#include "common.h"
#include "dirk_thread.h"
#include "commander.h"
//#include "serial_port.h"	// this is the library needed to connect with the arduino
#include "libs/serial/serial_port.h"

using std::string;
using namespace std;

/* this is to help with some of the rotation logic */
bool d_in_rotation = false;

/* keep track of the previous hunt state, as hunt state is sent periodically, not just on updates */
uint8_t d_prev_hunt_state = 10;

/* whether or not we are currently rotating */
bool d_rotating = false;

/* whether or not we are currently moving */
bool d_moving = false;

/* microsecond timestamp of the previous loop iteration */
unsigned long d_prev_loop_timestamp = 0;



void d_update_state(uint8_t &new_state) {

	switch (new_state) {
	case TRACKING_HUNT_STATE_ROTATE:
		d_rotating = true;
		break;
	case TRACKING_HUNT_STATE_MOVE:
		d_moving = true;
		break;
	case TRACKING_HUNT_STATE_OFF:
		// finished = true;
		break;
	case TRACKING_HUNT_STATE_START:
		// starting = true;
		break;
	}
}

float get_arduino_line(int &afd) {

	char buf[10];
	int i = 0;
	while (true) {
		// read a single char at a time
		char in_char = read(afd, buf, 1);

		if (in_char == '\n') {
			buf[i] = 0;
		} else {
			buf[i] = in_char;
		}
		i++;
	}

	// convert the output to a float
	return atof(buf);
}


void *dirk_thread(void *param) {
	
	// retrieve the MAVInfo struct that is sent to this function as a parameter
	struct MAVInfo *uavData = (struct MAVInfo *)param;
	
	// some constants that all need to become parameters
	char *bearing_file_name = (char *) "bearing_pa.csv";
	
	const int baudrate = 115200;
	const char *dirk_uart = (char *) "/dev/ttyACM0";

	// connect to the arduino
	SerialPort arduino(common::logfile_dir, false);
	arduino.begin_serial(dirk_uart, baudrate);

	/* Open a file to write bearing calcs to */
	FILE *bearing_file = fopen(bearing_file_name, "a");
	if (bearing_file == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("Error opening bearing output file\n");
		return NULL;
	}

	bool move_pending = false;
	int num_measurements_received = 0;
	bool received_data = false;

	// buffer to read in the data from the arduino
	char buf[128];
	char cur_char[1];
	int i = 0;

	// main loop that is constantly waiting for information from the arduino
	// this is all the measured bearing information coming in
	while (common::RUNNING_FLAG) {

		// handle hunt state changes required (sending of commands)
		if (uavData->tracking_status.hunt_mode_state != d_prev_hunt_state) {
			
			printf("State changed from %u to %u\n", d_prev_hunt_state, uavData->tracking_status.hunt_mode_state);

			if (uavData->tracking_status.hunt_mode_state == TRACKING_HUNT_STATE_WAIT) {

				// we want to make sure we have gotten 2 measurements from arduino before we send the next move command
				move_pending = true;
				num_measurements_received = 0;

				// we only care to send a set of move commands, so will need to adjust the send next command function

			} else {
				d_update_state(uavData->tracking_status.hunt_mode_state);
			}
			
			
			// update the prev hunt state to be this new state
			d_prev_hunt_state = uavData->tracking_status.hunt_mode_state;
			printf("Prev State changed to: %u\n", d_prev_hunt_state);
		}
	
		// check for data from the arduino
		// TODO: implement:		get_arduino_data();

		int num_bytes = 1;

		// printf("before here\n");

		// check how many bytes are waiting to be read
		ioctl(arduino.fd, FIONREAD, &num_bytes);

		// printf("number of available bytes: %d\n", num_bytes);

		if (num_bytes > 0) {
			// printf("before read\n");
			int n = read(arduino.fd, cur_char, 1);
			
			if (cur_char[0] == '\n') {
				// printf("newline\n");
				if (i > 2) {
					buf[i-1] = 0;
					
					// extract the bearing
					double bearing = atof(buf);

					printf("Current bearing: %f\n", bearing);
					
					// write the bearing to a file
					fprintf(bearing_file, "%llu, %i,%i,%f,%f\n", uavData->sys_time_us.time_unix_usec, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, bearing);

					// send the bearing message to the ground
					// send_bearing_message( bearing, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

					// just some additional handling for wanting to send move commands
					/*
					if (move_pending) {
						received_data = true;
					}
					*/
				}
				i = 0;
			} else {
				buf[i] = cur_char[0];
				i ++;
			}
		}


		if (move_pending && received_data) {
			num_measurements_received++;
			received_data = false;

			if (num_measurements_received >= 2) {
				// TODO: send the next move command here!!
				printf("sending a move command\n");
				// sendMoveCommand();

				// reset our information for whether or not we want to move
				move_pending = false;
				num_measurements_received = 0;
			}
		}
		
		// give the script a pause
		usleep(30000);
	}
	
	
	/* Be sure to close the output file and connection */
	fclose(bearing_file);
	arduino.end_serial();

	return NULL;
}
