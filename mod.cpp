/*
 * mod.cpp
 *
 *  Created on: August, 2014
 *      Author: Adrien Perkins
 */
#include "common.h"

#include "read_thread.h"
#include "wifly_thread.h"
#include "wifly2_thread.h"
#include "mod.h"
#include "dirk_thread.h"

using std::string;
using namespace std;


// variable declarations

bool verbose = false;	// default verbose to false
bool debug = false;		// default debug to false
bool nowifly = false;	// default to wanting wifly
bool get_commands = false;	// default for whether or not we want to read the command file
bool dual_wifly = false;	// default to only have one wifly active
bool phased_array = false;	// default to not using a phased array antenna
bool emily = false;			// default to not running the emily antenna configuration

bool execute_tracking = false;	// default to not executing a tracking mission
float flight_alt = 380;			// default flight is AMSL

MAVInfo uav;			// object to hold all the state information on the UAV

MavlinkSerial* pixhawk = nullptr;	// serial connection to the pixhawk
SerialPort* df_arduino = nullptr;	// serial connection to the df arduino

int RUNNING_FLAG = 1;	// default the read and write threads to be running

char* wifly_port1;
char* wifly_port2;
char* pa_port;

char* command_file = (char*) "commands";

/**
 * read in the passed arguments to the function on start
 *
 * TODO: figure out what to return and how to handle uart and baud values
 */
void read_arguments(int argc, char **argv, char **uart_name, int *baudrate, char **wifly1, char **wifly2) {

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
				cout << "nope\n";
				//printf(commandline_usage, argv[0], *uart_name, *baudrate);
				//throw EXIT_FAILURE;
			}
		}

		/* baud rate */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				*baudrate = atoi(argv[i + 1]);

			} else {
				cout << "more nope\n";
				//printf(commandline_usage, argv[0], *uart_name, *baudrate);
				//throw EXIT_FAILURE;
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

		/* wifly state */
		if (strcmp(argv[i], "-nw") == 0 || strcmp(argv[i], "--nowifly") == 0) {
			nowifly = true;
		}

		/* command file state */
		if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--commands") == 0) {
			get_commands = true;
			
			// get the name of the desired command file
			if (argc > i + 1) {
				command_file = argv[i + 1];
			}
		}

		/* wifly 1 port */
		if (strcmp(argv[i], "-w1") == 0 || strcmp(argv[i], "--wifly1") == 0) {
			*wifly1 = argv[i + 1];
		}

		/* wifly 2 port */
		if (strcmp(argv[i], "-w2") == 0 || strcmp(argv[i], "--wifly2") == 0) {
			*wifly2 = argv[i + 1];
			dual_wifly = true;
		}

		/* phased array port */
		if (strcmp(argv[i], "-pa") == 0 || strcmp(argv[i], "--phased") == 0) {
			phased_array = true;
		}

		/* whether or not executing a tracking mission */
		if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--track") == 0) {
			execute_tracking = true;
		}

		/* whether or not using the emily configuration */
		if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--emily") == 0) {
			emily = true;
		}
	}
}


void quit_handler(int sig) {

	// notify user of termination
	printf("Terminating script\n");

	// set the running flag to 0 to kill all loops
	RUNNING_FLAG = 0;

	pixhawk->end_serial();

}



int main(int argc, char **argv) {

	cout << "[MOD] starting...\n";
	printf("[MOD] printf starting...\n");

	// ids of the threads
	pthread_t readId;
	pthread_t wiflyId;
	pthread_t phasedId;
	pthread_t wifly2Id;

	/* default values for arguments */
	char *uart_name = (char*)"/dev/ttyUSB1";

	wifly_port1 = (char*) "/dev/ttyUSB0";
	wifly_port2 = (char*) "/dev/ttyUSB2";


	char* df_uart_name = (char*) "/dev/ttyACM0";

	int baudrate = 115200;

	cout << "[MOD] reading arguments\n";
	// read the input arguments
	read_arguments(argc, argv, &uart_name, &baudrate, &wifly_port1, &wifly_port2);

	// open and configure the com port being used for communication
	// begin_serial(uart_name, baudrate);
	cout << "[MOD] creating new mavlink serial object\n";
	pixhawk = new MavlinkSerial(verbose, uart_name, baudrate);

	cout << "[MOD] pixhawk fd " << pixhawk->fd << "\n";
	cout << "[MOD] pixhawk get fd " << pixhawk->get_fd() << "\n";

	// also create connection to df arduino if needed here
	if (emily) {
		df_arduino = new SerialPort(verbose, df_uart_name, baudrate);
	}

	// setup termination using CRT-C
	signal(SIGINT, quit_handler);


	// need to create read and write threads
	cout<< "[MOD] handling threads\n";
	pthread_create(&readId, NULL, read_thread, (void *)&uav);
	
	// create a thread for the wifly stuff (only if want wifly running)
	if (!nowifly && !phased_array) {
		printf("[MOD] starting wifly thread...\n");
		pthread_create(&wiflyId, NULL, wifly_thread, (void *)&uav);
	}

	if (dual_wifly) {
		printf("[MOD] starting wifly2 thread...\n");
		pthread_create(&wifly2Id, NULL, wifly2_thread, (void *) &uav);
	}

	if (phased_array) {
		printf("[MOD] starting dirk antenna thread...\n");
		pthread_create(&phasedId, NULL, dirk_thread, (void *)&uav);
	}

	pthread_join(readId, NULL);
	
	if (!nowifly && !phased_array) {
		pthread_join(wiflyId, NULL);
	}

	if (dual_wifly) {
		pthread_join(wifly2Id, NULL);
	}

	if (phased_array) {
		pthread_join(phasedId, NULL);
	}
	
	// close the pixhawk connection
	pixhawk->end_serial();

	return 0;
}

