/*
 * serial_port.h
 *
 * A class to be able to handle serial communication and connection, etc
 *
 *  Created on: Jul 6, 2015
 *      Author: Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <cstdlib>
#include <string>
#include <stdio.h>
#include <stdarg.h>


class  SerialPort {

public:
	/* file descriptor of the serial connection */
	int fd;
	//int fd2;
	//int num_conns;

	/* constructor */
	SerialPort(std::string logfile_dir);
	SerialPort(std::string logfile_dir, bool verbose);
	SerialPort(std::string logfile_dir, bool verbose, const char* &uart_name);
	SerialPort(std::string logfile_dir, bool verbose, const char* &uart_name, const int &baudrate);
	//SerialPort(bool verbose, int num_connections);

	/* destructor */
	~SerialPort();

	/* begin the serial connection (includes opening and configuring the connection) */
	void begin_serial(const char* &uart_name, const int &baudrate);

	/* simply open the serial connection, used for the more simple serial connections */
	void open_serial(const char* &uart_name);
	//void open_serial(char* &uart_name1, char* &uart_name2);
	
	/* end the serial connection */
	void end_serial();

protected:
	/* state variables */
	bool _verbose;
	FILE *_output_logfile;

	/* open com port */
	void open_port(const char* port);
	//void open_ports(const char* port1, const char* port2);

	/* close the port */
	void close_port();

	/* setup the com port */
	bool setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);


	inline void LOG_STATUS(std::string msg, ...) {
		// add a newline
		msg += "\n";

		// get the rest of the arguments
		va_list args;
		va_start(args, msg);

		// output
		vprintf(msg.c_str(), args);						// to command line
		msg.insert(0, "[STATUS]");
		vfprintf(_output_logfile, msg.c_str(), args);	// to logfile

		// close the end of the arguments
		va_end(args);
	}


	inline void LOG_DEBUG(std::string msg, ...) {
		// add a newline
		msg += "\n";

		// get the rest of the arguments
		va_list args;
		va_start(args, msg);

		// output
		if (_verbose) vprintf(msg.c_str(), args);		// to command line
		msg.insert(0, "[DEBUG]");
		vfprintf(_output_logfile, msg.c_str(), args);	// to logfile

		// close the end of the arguments
		va_end(args);
	}


	inline void LOG_ERROR(std::string msg, ...) {
		// add a newline
		msg += "\n";
		msg.insert(0, "[ERROR]");

		// get the rest of the arguments
		va_list args;
		va_start(args, msg);

		// output
		vprintf(msg.c_str(), args);						// to command line
		vfprintf(_output_logfile, msg.c_str(), args);	// to logfile

		// close the end of the arguments
		va_end(args);
	}

};


#endif /* SERIAL_PORT_H_ */
