// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>

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

#include "serialwifly.h" 	// this is from Louis' wifly code, which will be a library
#include "common.h"
#include "wifly_thread.h"
#include "test.h" 			// for bearing calculation
#include "commander.h"

using std::string;
using std::vector;
using namespace std;

/* this is to help with some of the rotation logic */
bool in_rotation = false;


int wifly_connect(char *port) {
	
	/* Start the connection to the WiFly */
	/* If the connection does not exist, quit */
	/* This is assuming we want to connect to /dev/ttyUSB0 */
	/* TODO: Don't make that assumption */
	int fd = start_connection(port);
	if (fd < 0)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("Error connecting to wifly. Exiting...\n");
		return 0;
	}
	
	return fd;
}


void *wifly_thread(void *param) {
	
	// retrieve the MAVInfo struct that is sent to this function as a parameter
	struct MAVInfo *uavData = (struct MAVInfo *)param;
	
	// some constants that all need to become parameters
	int num_samples = 1;
	char *ssid = (char *) "JAMMER01";
	char *file_name = (char *) "wifly.csv";
	char *bearing_file_name = (char *) "bearing_calc.csv";
	char *port = (char *) "/dev/ttyUSB0";
	
	// connect to the wifly
	int wifly_fd = wifly_connect(port);

	if (wifly_fd < 0) {
		printf("Error opening wifly connection\n");
		return NULL;
	}
	
	/* Go into command mode */
	commandmode(wifly_fd);

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
	
	vector<double> angles;
	vector<double> gains;


	// main loop that should be constantly taking measurements
	// until the main program is stopped
	while (RUNNING_FLAG) {

		/* check if we are in an official rotation */
		if (rotating) {
			if (!in_rotation) {
				// set our logic to mark we are now running the rotation logic
				in_rotation = true;

				// clear the vectors
				angles.clear();
				gains.clear();
			}

			angles.push_back((double) uavData->vfr_hud.heading);
			gains.push_back(scanrssi(wifly_fd, ssid));

		}

		if (!rotating && in_rotation) {
			in_rotation = false;

			// do bearing calculation at this point
			double bearing = get_bearing(angles, gains);

			// write the lat, lon, alt and bearing to file
			fprintf(bearing_file, "%i,%i,%f,%f\n", uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, bearing);

			// send a mavlink message of the calculated bearing
			send_bearing_message(bearing, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);

			// tell pixhawk we are finished with the rotation
			// send_finish_command();
		}
		
		/* Scan values to this file */
		/* Add degree at which you measure first */
		cout << uavData->vfr_hud.heading << " ";
		fprintf(wifly_file, "%u,%i,%i,%i,%f,", uavData->custom_mode, uavData->vfr_hud.heading, uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt);
		scanrssi_f(wifly_fd, ssid, wifly_file, num_samples);

		// TODO: send mavlink message of calculated rssi...
		
		/* sleep for some time before making another measurement (30 ms for now) */
		usleep(30000);
		
	}
	
	
	/* Be sure to close the output file and connection */
	fclose(wifly_file);
	close(wifly_fd);

	return NULL;
}
