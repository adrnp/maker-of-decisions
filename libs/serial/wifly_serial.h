/*
 * wifly_serial_class.h
 *
 * Builds on the serial class. Class to handle wifly specific communication
 *
 *  Created on: Jul 6, 2015
 *      Author: Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef WIFLY_SERIAL_H_
#define WIFLY_SERIAL_H_

// include the superclass SerialPort
#include "serial_port.h"

class WiflySerial : public SerialPort {


public:

	/* constructor */
	WiflySerial(std::string logfile_dir);
	WiflySerial(std::string logfile_dir, bool verbose);
	WiflySerial(std::string logfile_dir, bool verbose, const char* &uart_name);

	/* destructor */
	~WiflySerial();

	/* Ensure that you are in command mode */
	void enter_commandmode();

	/* Scan the channels for a specific SSID */

	// scan to read the rssi value given 1 measurement
	int scanrssi(char* ssid);

	// temporary function for debug purposes
	int scanrssi2(char* ssid);

	// scan a given number of times and right each to a file
	int scanrssi_f(char *ssid, FILE *f, int numtimes);

private:

	/* Parses output from scan  */
	int getrssi(char *, char *);

	/* new function to parse the output from a scan */
	int parserssi(char* buf, char* ssid);

};


#endif /* WIFLY_SERIAL_H_ */
