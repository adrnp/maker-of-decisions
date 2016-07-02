// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
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

// include class definition
#include "wifly_serial.h"

// TODO: Investigate this parameter... what exactly does it do?
#define WD 1000
#define TIMEOUT 2	// wifly read loop timeout in [s]

using std::string;
using namespace std;

WiflySerial::WiflySerial(std::string logfile_dir) : SerialPort(logfile_dir) {
	printf("simple wifly constructor\n");
}


WiflySerial::WiflySerial(std::string logfile_dir, bool verbose) : SerialPort(logfile_dir, verbose) {
	// nothing specific to do in this constructor
	printf("verbose wifly constructor\n");
}

WiflySerial::WiflySerial(std::string logfile_dir, bool verbose, const char* &uart_name) : SerialPort(logfile_dir, verbose, uart_name) {
	printf("complex wifly constructor\n");
}

WiflySerial::~WiflySerial() {
	// nothing to do in the destructor
}


void WiflySerial::enter_commandmode() {

	LOG_DEBUG("entering command mode...");

	// First, we must make sure we are in command mode
	// For now, let's assume we just booted up
	/*
	write(fd, "exit\r", 5);
	usleep(WD);
	write(fd, "exit\r", 5);
	usleep(WD);
	*/

	/* Give it the turn on command */
	/* Must wait 250 ms before and after giving $$$ command */
	/* To be safe, I wait 300 ms */
	write(fd, "$$$", 3);
	usleep(300000);
}

/**
 * Scans the channels and returns the rssi of the specified SSID
 */
int WiflySerial::scanrssi(char *ssid) {
	
	// the return value is the RSSI value of interest
	int rssi_value = INT_MAX;

	LOG_DEBUG("scanning...");
	
	write(fd, "scan 20\r", 8);
	usleep(500000);
	LOG_DEBUG("reading from wifly...");

	/* set a check on the connection to make sure can read */
	fd_set set;
	FD_ZERO(&set);
	FD_SET(fd, &set);

	// the timeout for the checking
	struct timeval timeout;
	timeout.tv_sec = TIMEOUT;
	timeout.tv_usec = 0;

	// do the actual checking
	int rv = select(fd + 1, &set, NULL, NULL, &timeout);
	if(rv == -1) {	// error
		LOG_ERROR("[WIFLY] unable to read from wifly");
	} else if(rv == 0) {	// timeout
		LOG_ERROR("[WIFLY] wifly read timeout");
	} else {	// read available
		
		// LD: valgrind complained of uninitialized values here
		//char buf[2048];
		char buf[2048] = "";
		read( fd, buf, sizeof(buf) );
		LOG_DEBUG("%s", buf);

		rssi_value = parserssi(buf, ssid);
	}
	
	return rssi_value;
}



// private functions


/*
 * New function for parsing message, ideally prevents seg faults
 */
int WiflySerial::parserssi(char* buf, char* ssid) {
	LOG_DEBUG("parsing rssi from message...");

	char *delim = (char *) "\n";
	char *comma_delim = (char *)  ",";
	char *jam_token;
	char *pEnd;
	int i = 0;
	int rssi_value = INT_MAX;

	char *token = strtok(buf, delim);

	// get to the starting point of the message
	while (token != NULL && strstr(token, (char*) "SCAN:Found") == NULL) {
		token = strtok(NULL, delim);
	}

	// can get the information of how many entries found
	//jam_token = strtok(token, " ");
	//jam_token = strtok(NULL, " ");
	//LOG_DEBUG("found %s results\n", jam_token);

	// get to the first actual line of the scan results
	token = strtok(NULL, delim);

	// now loop through to the end of the message
	while (token != NULL && strstr(token, (char*) "END:") == NULL) {

		LOG_DEBUG("%s\n", token);
		
		// check to see if this row contains the desired SSID
		if (strstr(token, ssid) != NULL) {
			jam_token = strtok(token, comma_delim);

			while (jam_token != NULL && i < 2) {
				jam_token = strtok(NULL, comma_delim);
				i++;
			}
			i = 0;

			// jam_token should now contain the rssi value
			// need to make sure it's not null and is an int
			if (jam_token != NULL) {
				rssi_value = (int) strtol(jam_token, &pEnd, 10);

				LOG_STATUS("rssi value parsed: %i", rssi_value);

				// check to make sure this was indeed the rssi value
				if (strstr(pEnd, "\0") == NULL) {
					LOG_STATUS("rssi not correct value");
					rssi_value = INT_MAX;
				}
			}

			// set token to null to end search
			token = NULL;
		} else {
			// get the next line
			token = strtok(NULL, delim);
		}
	}

	// return the rssi value
	return rssi_value;
}



