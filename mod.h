/*
 * mod.h
 *
 *  Created on: Aug 18, 2014
 *      Author: adrienp
 */

#ifndef MOD_H_
#define MOD_H_

// all the standard includes
// include "common_include.h"


// include common variables
// #include "common_vars.h"


/* read in the command line arguments */
void read_arguments(int argc, char **argv, char **uart_name, char *baudrate, char **wifly1, char **wifly2);

/* quit using CTRL-C */
void quit_handler(int sig);


/* main function */
int main(int argc, char **argv);


// just putting this here as a placeholder for now
// void generateCommand();


#endif /* MOD_H_ */
