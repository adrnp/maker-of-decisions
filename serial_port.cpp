/*
 * serial_port.cpp
 *
 *  Created on: Jan 23, 2015
 *      Author: adrienp
 */


#include "serial_port.h"	// serial port header file
#include "common.h"			// set of common variables to be used throughout


// variables to be accessible by other functions
/** id of the serial port that has been opened and that want to access **/
int fd;



/**
 * function to open and configure the com port to be used.
 *
 */
void begin_serial( char* &uart_name, const int &baudrate) {

	// display status as needed
	if (verbose) printf("Trying to connect to %s.. ", uart_name);
	fflush(stdout);

	// attempt to open the com port
	fd = _open_port(uart_name);

	// check for success, display comments as needed
	if (fd == -1) {
		if (verbose) printf("failure, could not open port.\n");
		throw EXIT_FAILURE;
	} else {
		if (verbose) printf("success.\n");
	}

	// display status as needed
	if (verbose) printf("Trying to configure %s.. ", uart_name);

	// attempt to setup the com port
	bool setup = _setup_port(baudrate, 8, 1, false, false);

	// check success, display comments as needed
	if (!setup) {
		if (verbose) printf("failure, could not configure port.\n");
		throw EXIT_FAILURE;
	} else {
		if (verbose) printf("success.\n");
	}

	// error checking, display comments as needed
	if (fd <= 0) {
		if (verbose) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		throw EXIT_FAILURE;
	} else {
		if (verbose) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	}

	// final status update to display as needed
	if (verbose) printf("\nREADY, waiting for heartbeat.\n");

}

/**
 * end the serial connection currently being used
 */
void end_serial() {
	_close_port();
}



/**
 * open the desired serial port
 *
 * returns the file descriptor int
 */
int _open_port(const char* port)
{
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	// error checking
	if (fd == -1) {
		/* Could not open the port. */
		return(-1);
	}

	// finalize opening of the serial port
	fcntl(fd, F_SETFL, 0);

	// return the file descriptor
	return (fd);
}


/**
 * close the serial port
 */
void _close_port()
{
	// close the port, if it is a valid open port
	if (fd >= 0) {
		close(fd);
	}
}


/**
 * an ideally private function to help set up the serial port to be
 * used for communcation with the vehicle (through the antenna)
 *
 * returns true of successfully set up, false otherwise
 * will also print out errors to the console
 */
bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{

	// check the file descriptor
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}

	// read file descriptor config
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}


	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
			INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
			ONOCR | OFILL | OPOST);

#ifdef OLCUC
	config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
	config.c_oflag &= ~ONOEOT;
#endif


	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);


	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;


	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	// struct terminos options;
	// tcgetattr(fd, &options);

	// configure for the desired baud rate
	switch (baud)
	{
	case 1200:
		if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 1800:
		cfsetispeed(&config, B1800);
		cfsetospeed(&config, B1800);
		break;
	case 9600:
		cfsetispeed(&config, B9600);
		cfsetospeed(&config, B9600);
		break;
	case 19200:
		cfsetispeed(&config, B19200);
		cfsetospeed(&config, B19200);
		break;
	case 38400:
		if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 57600:
		if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 115200:
		if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
	case 460800:
		if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 921600:
		if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	default:
		fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
		return false;

		break;
	}


	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}

	// successfully set up the port
	return true;
}

