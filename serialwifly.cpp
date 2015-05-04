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

#include "serialwifly.h" 

using std::string;
using namespace std;

// TODO: Investigate this parameter... what exactly does it do?
#define WD 1000


/**
 * Start serial connection over one some port
 *
 * *port should be of the format "/dev/ttyUSB0"
 */
int start_connection(char *port)
{
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	return fd;
}


/**
 * This puts you in command mode
 * Must wait 250 ms before
 */
int commandmode(int fd)
{
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

	return 0;
}


/**
 * Parses output from scan command.
 * Searches for SSID provided.
 * Returns the RSSI value of that SSID, as an int.
 */
int getrssi(char *buf, char *ssid)
{
	char *delim = (char *) "\n";
	char *token = strtok(buf, delim);
	int rssi_value = INT_MAX;

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

/**
 * Scans the channels and returns the rssi of the specified SSID
 */
int scanrssi(int fd, char *ssid)
{
	char buf[1000];
	write(fd, "scan 10\r", 8);
	usleep(1000000);
	read(fd, buf, sizeof(buf));

	return getrssi(buf, ssid);
}


/**
 * Scans numtimes times and prints rssi values to a line in a file
 */
int scanrssi_f(int fd, char *ssid, FILE *f, int numtimes)
{
	int rssi_value, i;
	for (i = 1; i <= (numtimes-1); i++)
	{
		rssi_value = scanrssi(fd, ssid);
		fprintf(f, "%i,", rssi_value);
		//printf("rssi_value = %i\n", rssi_value);
	}

	/* Don't include comma for last one */
	rssi_value = scanrssi(fd, ssid);
	fprintf(f, "%i\n", rssi_value);
	//printf("rssi_value = %i\n", rssi_value);

	return 0;
}


/**
 * Scans numtimes times and prints rssi values to a line in a file
 */
int scanrssi_2(int fd1, int fd2, char *ssid, FILE *f1, FILE *f2, int numtimes)
{
	int rssi_1, rssi_2, i;
	for (i = 1; i <= (numtimes-1); i++)
	{
		rssi_1 = scanrssi(fd1, ssid);
		rssi_2 = scanrssi(fd2, ssid);
		fprintf(f1, "%i,", rssi_1);
		fprintf(f2, "%i,", rssi_2);
		printf("Wifly 1 RSSI = %i\n", rssi_1);
		printf("Wifly 2 RSSI = %i\n", rssi_2);
	}

	/* Don't include comma for last one */
	rssi_1 = scanrssi(fd1, ssid);
	rssi_2 = scanrssi(fd2, ssid);
	fprintf(f1, "%i\n", rssi_1);
	fprintf(f2, "%i\n", rssi_2);
	printf("Wifly 1 RSSI = %i\n", rssi_1);
	printf("Wifly 2 RSSI = %i\n", rssi_2);

	return 0;
}





