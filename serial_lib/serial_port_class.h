/*
 * serial_port_class.h
 *
 * A class to be able to handle serial communication and connection, etc
 *
 *  Created on: Jul 6, 2015
 *      Author: Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef SERIAL_PORT_CLASS_H_
#define SERIAL_PORT_CLASS_H_


class  SerialPort {

public:
	/* file descriptor of the serial connection */
	int fd;
	//int fd2;
	//int num_conns;

	/* constructor */
	SerialPort(bool verbose);
	//SerialPort(bool verbose, int num_connections);

	/* destructor */
	~SerialPort();

	/* begin the serial connection (includes opening and configuring the connection) */
	void begin_serial(char* &uart_name, const int &baudrate);

	/* simply open the serial connection, used for the more simple serial connections */
	void open_serial(char* &uart_name);
	//void open_serial(char* &uart_name1, char* &uart_name2);
	
	/* end the serial connection */
	void end_serial();

private:
	/* state variables */
	bool _verbose;

	/* open com port */
	void open_port(const char* port);
	//void open_ports(const char* port1, const char* port2);

	/* close the port */
	void close_port();

	/* setup the com port */
	bool setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);


};


#endif /* SERIAL_PORT_CLASS_H_ */
