// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>

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


#include "mavlink_serial.h"

MavlinkSerial::MavlinkSerial() : SerialPort() {
	printf("default mavlink serial constructor\n");
}

MavlinkSerial::MavlinkSerial(bool verbose) : SerialPort(verbose) {
	// nothing specific to do in this constructor
	printf("verbose mavlink serial constructor\n");
}

MavlinkSerial::MavlinkSerial(bool verbose, char* &uart_name,  const int &baudrate) : SerialPort(verbose, uart_name, baudrate) {
	printf("complex mavlink serial constructor\n");
	printf("fd = %i\n", fd);
}

MavlinkSerial::~MavlinkSerial() {
	// nothing to do in the destructor
}

int MavlinkSerial::get_fd() {
	printf("get fd = %i\n", fd);
	return fd;
}


uint8_t MavlinkSerial::read_serial(mavlink_status_t *lastStatus, mavlink_message_t *message) {
	// variables needed for the message reading
	uint8_t cp;					// not sure
	mavlink_status_t status;	// current message status
	uint8_t msgReceived = false; // whether or not a message was correctly received

	// read in from the file
	if (read(fd, &cp, 1) > 0) {

		// Check if a message could be decoded, return the message in case yes
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, message, &status);

		// check the packet drop count to see if there was a packet dropped during this message reading
		if (lastStatus->packet_rx_drop_count != status.packet_rx_drop_count) {

			// print out some error information containing dropped packet indo
			if (_verbose) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			
			// print out the characters of the packets themselves
			if (_verbose) {
				unsigned char v=cp;
				fprintf(stderr,"%02x ", v);
			}
		}

		// update the last message status 
		*lastStatus = status;

	} else { // means unable to read from the serial device

		// print out error as needed
		if (_verbose) fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
	}

	// return whether or not the message was received
	return msgReceived;
}


int MavlinkSerial::write_serial(mavlink_message_t &message) {

	if (_verbose) printf("writing to pixhawk...\n");

	// buffer needed for mavlink msg and function
	char buf[300];

	// Send message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write packet via serial link
	write(fd, buf, len);

	// Wait until all data has been written
	tcdrain(fd);

	if (_verbose) printf("wrote message of length %u\n", len);

	return len;


}
