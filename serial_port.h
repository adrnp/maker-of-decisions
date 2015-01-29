/**
 * @file serial_port.h
 *
 * @brief Serial Interface Handling
 *
 * functions for setting up, opening and closing the serial port
 *
 *
 */

#ifndef SEIRAL_PORT_H_
#define SERIAL_PORT_H_

// some includes

#include <fcntl.h>    /* File control definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <cstdio>
 #include <cstdlib>


// variables to be accessible by other functions

/** id of the serial port that has been opened and that want to access **/
int fd;


// function definitions

/* open and setup the com port */
void begin_serial(const char* &uart_name, const char* &buadrate);

/* close out the serial connection */
void end_serial();


// TODO: figure out how to correctly declare the following as private

/* open com port */
int _open_port(const char* port);

/* close the port */
void _close_port();

/* setup the com port */
bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);









#endif /* SERIAL_PORT_H_ */
