
#ifndef WIFLY_THREAD_H_
#define WIFLY_THREAD_H_


/* update the booleans that keep track of the current state */
void update_state(uint8_t &new_state);

/* main thread function to be called by pthread in mod.cpp */
void *wifly_thread(void *param);


#endif /* WIFLY_THREAD_H_ */
