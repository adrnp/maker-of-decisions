/*
 * mod.cpp
 *
 *  Created on: August, 2014
 *      Author: Adrien Perkins
 */

// include mavlink message structure ... should move it over to .h
#include <jager/mavlink.h>


#include "mod.h"
#include "port_setup.h"
#include "read_thread.h"
#include "read_thread.cpp"

using std::string;
using namespace std;


int main(int argc, char **argv) {

	// ids of the threads
	pthread_t readId;

	/* default values for arguments */
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 115200;
	const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [options]\n\n"
			"\t-v/--verbose\t\t detailed output of current state\n"
			"\n\t\tdefault: -d %s -b %i\n";

	/* read program arguments */
	int i;

	for (i = 1; i < argc; i++) { /* argv[0] is "mavlink" */
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf(commandline_usage, argv[0], uart_name, baudrate);
			return 0;
		}

		/* UART device ID */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}

		/* baud rate */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}

		/* verbosity */
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			// TODO: add levels of verbosity
			silent = false;
			verbose = true;
			printAll = true;
		}

	}

	// SETUP SERIAL PORT

	// Exit if opening port failed
	// Open the serial port.
	if (!silent) printf("Trying to connect to %s.. ", uart_name);
	fflush(stdout);

	fd = open_port(uart_name);
	if (fd == -1)
	{
		if (!silent) printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}
	if (!silent) printf("Trying to configure %s.. ", uart_name);
	bool setup = setup_port(fd, baudrate, 8, 1, false, false);
	if (!setup)
	{
		if (!silent) printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}

	int noErrors = 0;
	if (fd == -1 || fd == 0)
	{
		if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	}

	if(fd < 0)
	{
		exit(noErrors);
	}

	// Run indefinitely while the serial loop handles data
	if (!silent) printf("\nREADY, waiting for heartbeat.\n");


	// need to create read and write threads
	pthread_create(&readId, NULL, serial_read, (void *)&uav);


	pthread_join(readId, NULL);


	close_port(fd);

	return 0;
}

