/*
 * mod.cpp
 *
 *  Created on: August, 2014
 *      Author: Adrien Perkins
 */

#include "mod.h"
#include "serial_port.h"

#include "common.h"

using std::string;
using namespace std;


// variable declarations

bool verbose = false;  // default verbose to false
bool debug = false;    // default debug to false


/**
 * read in the passed arguments to the function on start
 *
 * TODO: figure out what to return and how to handle uart and baud values
 */
void read_arguments(int argc, char **argv, char **uart_name, int *baudrate) {

	// string to be displayed on incorrect inputs to show correct function usage
	const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [options]\n\n"
			"\t-v/--verbose\t\t detailed output of current state\n"
			"\n\t\tdefault: -d %s -b %i\n";

	// loop through all the program arguments
	for (int i = 1; i < argc; i++) { /* argv[0] is "mavlink" */

		// help text requested
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf(commandline_usage, argv[0], *uart_name, *baudrate);
			throw EXIT_FAILURE;
		}

		/* UART device ID */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				*uart_name = argv[i + 1];

			} else {
				printf(commandline_usage, argv[0], *uart_name, *baudrate);
				throw EXIT_FAILURE;
			}
		}

		/* baud rate */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				*baudrate = atoi(argv[i + 1]);

			} else {
				printf(commandline_usage, argv[0], *uart_name, *baudrate);
				throw EXIT_FAILURE;
			}
		}

		/* verbosity */
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}

		/* debug */
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--debug") == 0) {
			debug = true;
		}

	}
}



int main(int argc, char **argv) {

	// ids of the threads
	pthread_t readId;

	/* default values for arguments */
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 115200;

	// read the input arguments
	read_arguments(argc, argv, &uart_name, &baudrate);

	// open and configure the com port being used for communication
	begin_serial(uart_name, baudrate);


	// need to create read and write threads
	/*
	pthread_create(&readId, NULL, serial_read, (void *)&uav);


	pthread_join(readId, NULL);
	*/

	end_serial();

	return 0;
}

