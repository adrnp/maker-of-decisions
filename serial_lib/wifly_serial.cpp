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
	write(fd, "exit\r", 5);
	usleep(WD);
	write(fd, "exit\r", 5);
	usleep(WD);

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
	char buf[1000];
	write(fd, "scan 50\r", 8);
	usleep(1000000);
	if (_verbose) printf("reading from wifly...\n");
	read(fd, buf, sizeof(buf));

	return getrssi(buf, ssid);
}


/**
 * Scans numtimes times and prints rssi values to a line in a file
 */
int WiflySerial::scanrssi_f(char *ssid, FILE *f, int numtimes)
{
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

	if (_verbose) printf("starting get while loop...\n");
	while (token)
	{
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



