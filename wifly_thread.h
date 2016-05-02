/**
 * @file wifly_thread.h
 *
 * Thread to handle reading from one or two wifly modules.
 * 
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef WIFLY_THREAD_H_
#define WIFLY_THREAD_H_


#include <vector>

using std::vector;


/**
 * WiflyHunter class.
 *
 * Class that contains a main loop to read from the wifly modules.
 */
class WiflyHunter {

public:

	/**
	 * constructor.
	 */
	WiflyHunter(struct MAVInfo* uavData, bool verbose);

	/**
	 * desctructor.
	 */
	~WiflyHunter();

	/**
	 * the main loop running for the thread
	 * @return  0 when finished
	 */
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
	
	/**
	 * set all variables initializng a rotation.
	 */
	void rotation_init();

	/**
	 * process all measurements from the rotation.
	 */
	void rotation_completed();

	/**
	 * check and update current state if needed.
	 */
	void check_hunt_state();

	/**
	 * update current state
	 * @param new_state  the new state
	 */
	void update_state(const uint8_t &new_state);

	/**
	 * get the max rssi value from the set collected during a rotation
	 * @param  rssi_values  vector of rssi values from this rotation
	 * @return              the maximum signal strength from a given rotation
	 */
	int get_max_rssi(const vector<double> rssi_values);

};

/**
 * springboard to starting the main loop of the WiflyHunter class.
 * @param  param  mavstruct containing uav information
 * @return        standard return
 */
void *wifly_thread(void *param);


#endif /* WIFLY_THREAD_H_ */
