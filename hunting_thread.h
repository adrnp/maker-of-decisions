#ifndef HUNTING_THREAD_H_
#define HUNTING_THREAD_H_

#include <vector>

using std::vector;

class Hunter {

public:

	/* constructor and desctructor */
	Hunter(struct MAVInfo* uavData, bool verbose);
	~Hunter();

	/* this is actually the main loop that will be running */
	int main_loop();


private:
	struct MAVInfo* _jager;

	/* whether or not to be verbose in outputs */
	bool _verbose;

	/* vehicle state information */
	bool _in_rotation;
	bool _rotating;
	bool _moving;
	bool _send_next;

	/* direct state info */
	uint8_t _curr_hunt_state;
	uint8_t _prev_hunt_state;


	/* measurement data */
	vector<double> _angles;
	vector<double> _gains;
	vector<double> _omni_gains;
	vector<int> _norm_gains;
	int _dir_rssi;
	int _omni_rssi;
	int16_t _meas_heading;
	double _bearing_cc;
	double _bearing_max;
	int _max_rssi;


	// helper functions
	
	/* handle rotation logic and related computation */
	void rotation_init();
	void rotation_completed();

	/* measurement function */
	void get_measurement();
	
	/* update the state booleans */
	void check_hunt_state();
	void update_state(const uint8_t &new_state);

	/* get the max rssi value from the set collected during a rotation */
	int get_max_rssi(const vector<double> rssi_values);

};




/* main thread function to be called by pthread in mod.cpp - actually just a springboard for the main thread in the above class */
void *hunting_thread(void *param);


#endif /* WIFLY_THREAD_H_ */