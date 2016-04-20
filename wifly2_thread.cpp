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
#include "libs/serial/wifly_serial.h" // include the new class for handling a wifly
#include "common.h"
#include "wifly2_thread.h"
#include "libs/bearing/bearing.h"		// this should be all the functions required for bearing calculations

using std::string;
using std::vector;
using namespace std;

/* microsecond timestamp of the previous loop iteration */
unsigned long prev_omni_loop_timestamp = 0;

// global variables that wifly_thread needs to be able to acces
unsigned long omni_update_timestamp = 0;
int omni_rssi = INT_MAX;
int16_t heading_omni_pre = 0;
int16_t heading_omni_post = 0;



void *wifly2_thread(void *param) {
	
	// retrieve the MAVInfo struct that is sent to this function as a parameter
	struct MAVInfo *uavData = (struct MAVInfo *)param;
	
	// some constants that all need to become parameters
	char *ssid = (char *) "JAMMER01"; // "ADL"; // "JAMMER01";
	char *file_name2 = (char *) "wifly2.csv";

	// connect to the second wifly
	WiflySerial* wifly2 = new WiflySerial(verbose, wifly_port2);
	if (wifly2->fd < 0) {
		printf("[WIFLY2] Error opening wifly connection\n");
		return NULL;
	}

	
	/* Go into command mode */
	wifly2->enter_commandmode();


	/* Open a file to write values to */
	/* Appending values */
	FILE *wifly_file2 = fopen(file_name2, "a");
	if (wifly_file2 == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("[WIFLY2] Error opening wifly2 output file\n");
		return NULL;
	}

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
		if (prev_omni_loop_timestamp != 0 && (current_loop_time - prev_omni_loop_timestamp) < 30000) {
			continue;
		}
		prev_omni_loop_timestamp = current_loop_time;


		//-----------------------------------------------//
		// make measurement, and get start and end angle (for during the measurement)
		//-----------------------------------------------//

		// TODO: potentially capture all measurements at the same time as 
		// making the rssi measurement (i.e. lat, lon, alt)
		// not sure what the variability is from here to later on

		omni_rssi = INT_MAX;

		if (verbose) printf("[WIFLY2] scanning wifly 2...\n");
		heading_omni_pre = uavData->vfr_hud.heading;

		// get the omni rssi, update it and the timestamp of that update
		omni_rssi = wifly2->scanrssi(ssid);
		omni_update_timestamp = current_loop_time;
		
		if (verbose) printf("[WIFLY2] omni rssi received: %i\n", omni_rssi);
		
		heading_omni_post = uavData->vfr_hud.heading;

		//-----------------------------------------------//
		// save the measured data information to file
		//-----------------------------------------------//

		/* write the omni atenna information */
		printf("[WIFLY2] writing omni rssi to file: %i\n", omni_rssi);
		fprintf(wifly_file2, "%llu,%u,%i,%i,%i,%i,%i,%f,%i\n",
				uavData->sys_time_us.time_unix_usec, uavData->custom_mode, rotating, heading_omni_pre, heading_omni_post,
				uavData->gps_position.lat, uavData->gps_position.lon, uavData->vfr_hud.alt, omni_rssi);

	} // end of while running
	
	
	/* Be sure to close the output file and connection */
	fclose(wifly_file2);
	wifly2->end_serial();
	
	return NULL;
}
