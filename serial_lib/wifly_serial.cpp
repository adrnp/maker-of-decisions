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

using std::string;
using namespace std;

WiflySerial::WiflySerial() : SerialPort() {
	printf("simple wifly constructor\n");
}


WiflySerial::WiflySerial(bool verbose) : SerialPort(verbose) {
	// nothing specific to do in this constructor
	printf("verbose wifly constructor\n");
}

WiflySerial::WiflySerial(bool verbose, char* &uart_name) : SerialPort(verbose, uart_name) {
	printf("complex wifly constructor\n");
}

WiflySerial::~WiflySerial() {
	// nothing to do in the destructor
}


void WiflySerial::enter_commandmode() {

	if (_verbose) printf("entering command mode...\n");

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
	if (_verbose) printf("scanning...\n");
	char buf[2048];
	write(fd, "scan 10\r", 8);
	usleep(1000000);
	if (_verbose) printf("reading from wifly...\n");
	
	read(fd, buf, sizeof(buf));
	if (_verbose) printf("%s\n", buf);
	
	return parserssi(buf, ssid);
}


// temporary function TO BE DELTED
int WiflySerial::scanrssi2(char *ssid) {
	if (_verbose) printf("scanning in 2...\n");
	char buf[20480];
	write(fd, "scan 10\r", 8);
	usleep(1000000);
	if (_verbose) printf("reading from wifly...\n");
	
	read(fd, buf, sizeof(buf));
	if (_verbose) printf("%s\n", buf);
	
	/*
	while (read(fd, buf, sizeof(buf)) > 0) {
		if (_verbose) printf("%s\n", buf);
		//if (rssi == INT_MAX) {
		//	rssi = getrssi(buf, ssid);
		//}
	}
	*/
	return parserssi(buf, ssid);
	//return rssi;
}


/**
 * Scans numtimes times and prints rssi values to a line in a file
 */
int WiflySerial::scanrssi_f(char *ssid, FILE *f, int numtimes)
{
	numtimes = 1;
	if (_verbose) printf("scanning to write to file...\n");
	int rssi_value, i;
	for (i = 1; i <= (numtimes-1); i++)
	{
		rssi_value = scanrssi(ssid);
		fprintf(f, "%i,", rssi_value);
		//printf("rssi_value = %i\n", rssi_value);
	}

	/* Don't include comma for last one */
	rssi_value = scanrssi(ssid);
	fprintf(f, "%i\n", rssi_value);
	//printf("rssi_value = %i\n", rssi_value);

	return rssi_value;
}




// private functions


/**
 * Parses output from scan command.
 * Searches for SSID provided.
 * Returns the RSSI value of that SSID, as an int.
 */
int WiflySerial::getrssi(char *buf, char *ssid)
{
	if (_verbose) printf("getting rssi from message...\n");

	char *delim = (char *) "\n";
	char *token = strtok(buf, delim);
	int rssi_value = INT_MAX;

	//if (_verbose) printf("%s\n", buf);

	if (_verbose) printf("starting get while loop...\n");
	while (token)
	{
		if (_verbose) printf("%s\n", token);
		
		if (strstr(token, ssid) != NULL)
		{
			char *comma_delim = (char *)  ",";
			char *jam_token = strtok(token, comma_delim);
			jam_token =	strtok(0, comma_delim);
			jam_token =	strtok(0, comma_delim);
			rssi_value = atoi(jam_token);
			token = NULL;
		}
		else
		{
			token = strtok(0, delim);
		}
	}

	return rssi_value;
}


/*
 * New function for parsing message, ideally prevents seg faults
 */
int WiflySerial::parserssi(char* buf, char* ssid) {
	if (_verbose) printf("parsing rssi from message...\n");

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
	//if (_verbose) printf("found %s results\n", jam_token);

	// get to the first actual line of the scan results
	token = strtok(NULL, delim);

	// now loop through to the end of the message
	while (token != NULL && strstr(token, (char*) "END:") == NULL) {

		if (_verbose) printf("%s\n", token);
		
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

				printf("rssi value parsed: %i\n", rssi_value);

				// check to make sure this was indeed the rssi value
				if (strstr(pEnd, "\0") == NULL) {
					printf("rssi not correct value\n");
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



