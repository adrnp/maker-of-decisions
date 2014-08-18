/*
 * port_setup.h
 *
 *  Created on: Aug 4, 2014
 *      Author: adrienp
 */

#ifndef PORT_SETUP_H_
#define PORT_SETUP_H_

/** open com port */
int open_port(const char* port);

/* setup the com port */
bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

/* close the port */
void close_port(int fd);



#endif /* PORT_SETUP_H_ */
