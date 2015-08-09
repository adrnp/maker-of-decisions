
#ifndef DIRK_THREAD_H_
#define DIRK_THREAD_H_

/* main thread function to be called by pthread in mod.cpp */
void *dirk_thread(void *param);

/* parse the line received from the arduino */
void parse_message(char *buf);


#endif /* DIRK_THREAD_H_ */
