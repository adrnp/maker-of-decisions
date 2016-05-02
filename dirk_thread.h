/**
 * @file dirk_thread.h
 *
 * Thread handling reading from and controlling the beam steering antenna.
 * 
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef DIRK_THREAD_H_
#define DIRK_THREAD_H_


/* main thread function to be called by pthread in mod.cpp */
void *dirk_thread(void *param);


#endif /* DIRK_THREAD_H_ */
