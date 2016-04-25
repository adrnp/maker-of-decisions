
#ifndef WIFLY_THREAD_H_
#define WIFLY_THREAD_H_
#include <vector>

using std::vector;

class WiflyHunter {

public:

	/* constructor and desctructor */
	WiflyHunter(struct MAVInfo* uavData, bool verbose);
	~WiflyHunter();

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
	int16_t _heading_dir_pre;
	int16_t _heading_dir_post;
	int16_t _heading_omni_post;
	double _bearing_cc;
	double _bearing_max;
	int _max_rssi;


	// helper functions
	
	/* handle rotation logic and related computation */
	void rotation_init();
	void rotation_completed();

	/* update the state booleans */
	void check_hunt_state();
	void update_state(const uint8_t &new_state);

	/* get the max rssi value from the set collected during a rotation */
	int get_max_rssi(const vector<double> rssi_values);

};

/* main thread function to be called by pthread in mod.cpp */
void *wifly_thread(void *param);


#endif /* WIFLY_THREAD_H_ */
