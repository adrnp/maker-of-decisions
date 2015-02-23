
#ifndef WIFLY_THREAD_H_
#define WIFLY_THREAD_H_


/* create connection to the wify module */
int wifly_connect(char *port);

/* main thread function to be called by pthread in mod.cpp */
void *wifly_thread(void *param);


#endif /* WIFLY_THREAD_H_ */