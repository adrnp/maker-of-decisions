/*
 * mod.h
 *
 *  Created on: Aug 18, 2014
 *      Author: adrienp
 */

#ifndef MOD_H_
#define MOD_H_


/* quit using CTRL-C */
void quit_handler(int sig);

/* read in the config file */
int get_configuration(int argc, char **argv);


/* main function */
int main(int argc, char **argv);


// just putting this here as a placeholder for now
// void generateCommand();


#endif /* MOD_H_ */
