/*
 * mavlink_serial_class.h
 *
 * Builds on the serial class. Class to handle mavlink specific communication
 *
 *  Created on: Jul 7, 2015
 *      Author: Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef MAVLINK_SERIAL_CLASS_H_
#define MAVLINK_SERIAL_CLASS_H_

// include the superclass SerialPort
#include "serial_port_class.h"

#include <jager/mavlink.h>

class MavlinkSerial : public SerialPort {


public:

	/* constructor */
	MavlinkSerial(bool verbose);
	MavlinkSerial(bool verbose, char* &uart_name);

	/* destructor */
	~MavlinkSerial();

	/* function to write a mavlink message */
	int write_serial(mavlink_message_t &message);

	/* function to read a message */
	uint8_t read_serial(mavlink_status_t *lastStatus, mavlink_message_t *message);

private:
	bool _verbose;

};


#endif /* MAVLINK_SERIAL_CLASS_H_ */
