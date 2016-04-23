/*
 * mavlink_serial.h
 *
 * Builds on the serial class. Class to handle mavlink specific communication
 *
 *  Created on: Jul 7, 2015
 *      Author: Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef MAVLINK_SERIAL_H_
#define MAVLINK_SERIAL_H_

// include the superclass SerialPort
#include "serial_port.h"

#include <jager/mavlink.h>

class MavlinkSerial : public SerialPort {


public:

	/* constructor */
	MavlinkSerial();
	MavlinkSerial(bool verbose);
	MavlinkSerial(bool verbose, const char* &uart_name,  const int &baudrate);

	/* destructor */
	~MavlinkSerial();

	/* test function to get the file descriptor */
	int get_fd();

	/* function to read a message */
	uint8_t read_serial(mavlink_status_t *lastStatus, mavlink_message_t *message);
	

	// specific message functions (wrappers to the mavlink)
	
	/* send the next tracking command */
	void send_tracking_command(const float &north, const float &east, const float &alt);

	/* send a command to rotate the vehicle
	 * direction: -1 is CCW and 1 is CW
	 */
	void send_rotate_command(const float direction);

	/* send a command to go into off mode */
	void send_finish_command();

	/* send the bearing message */
	// TODO: combine these 2 bearing senders into 1
	void send_bearing_cc_message(const double &bearing, const int32_t &lat, const int32_t &lon, const float &alt);
	void send_bearing_mle_message(const double &bearing, const int32_t &lat, const int32_t &lon, const float &alt);

	/* send the rssi message */
	void send_rssi_message(const int &rssi, const int &rssi2, const int16_t &heading, const int32_t &lat, const int32_t &lon, const float &alt);

private:

	int _command_id;

	/* function to write a mavlink message */
	int write_serial(mavlink_message_t &message);

	

};


#endif /* MAVLINK_SERIAL_H_ */
