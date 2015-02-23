#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

#include "serialwifly.h" // this is from Louis' wifly code, which will be a library
#include "common.h"
#include "wifly_thread.h"


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
	char *ssid = "JAMMER01";
	char *file_name = "wifly.csv";
	char *port = "/dev/ttyUSB0";
	
	// connect to the wifly
	int wifly_fd = wifly_connect(port);
	
	/* Go into command mode */
	commandmode(wifly_fd);

	/* Open a file to write values to */
	/* Appending values */
	FILE *wifly_file = fopen(file_name, "a");
	if (f == NULL)
	{
		// TODO: figure out what we do want to return when there is an error
		printf("Error opening output file\n");
		return 0;
	}
	
	// main loop that should be constantly taking measurements
	// until the main program is stopped
	while (RUNNING_FLAG) {
		
		/* Scan values to this file */
		/* Add degree at which you measure first */
		fprintf(wifly_file, "%i,", uavData->vfr_hud.heading);
		scanrssi_f(wifly_fd, ssid, wifly_file, numtimes);
		
		/* sleep for some time before making another measurement (300 ms for now) */
		usleep(300000);
		
	}
	
	
	/* Be sure to close the output file and connection */
	fclose(f);
	close(fd);
	
}
