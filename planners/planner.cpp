#include <cstdlib>
#include <unistd.h>

#include "libs/bearing/bearing.h"
#include "planner.h"


Planner() :
_bearing_cc(0),
_bearing_max(0),
_bearing_max3(0),
_max_rssi(0)
{
	// make sure all the vectors have been cleared
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();
}


~Planner() {

}


void Planner::reset_observations() {
	
	/* clear the vectors to get ready for another observation set */
	_angles.clear();
	_gains.clear();
	_omni_gains.clear();
	_norm_gains.clear();

}


void Planner::complete_observations() {

	/* get bearing and values */
	_bearing_cc = get_bearing_cc(_angles, _gains);		// do bearing calculation at this point
	_bearing_max = get_bearing_max(_angles, _gains);	// also do max bearing calculation
	_max_rssi = get_max_rssi(_gains);					// get what the max value was for the rssi

}


void Planner::update_observation(const double &heading, const double &dir_gain, const double &omni_gain) {

	/* add heading and rssi to the correct arrays */
	_angles.push_back((double) heading);
	_gains.push_back(dir_gain);
	_omni_gains.push_back(omni_gain);

}

