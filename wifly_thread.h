
#ifndef WIFLY_THREAD_H_
#define WIFLY_THREAD_H_
#include <vector>

/* update the booleans that keep track of the current state */
void update_state(uint8_t &new_state);

/* loop through the list of rssi values and get the highest gain */
int get_max_rssi(std::vector<float> rssi_values);

/* main thread function to be called by pthread in mod.cpp */
void *wifly_thread(void *param);


#endif /* WIFLY_THREAD_H_ */
