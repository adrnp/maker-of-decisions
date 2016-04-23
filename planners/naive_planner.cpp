#include <vector>
#include "planner.h"

using std::vector;

#define BEARING_TOL 10.0			// desired tolerance in degrees for 2 bearing measurements to be considered the same
#define STEP_INCREASE_FACTOR 2.0	// factor by which step size increases if along the same path as before
#define STEP_SMALL 10.0
#define STEP_LARGE 50.0


class NaivePlanner : public Planner {

public:

	/* return the action */
	vector<float> action();

private:

	// overall information used to make decisions
	vector<double> _observed_bearing;
	vector<int> _observed_rssi;
	vector<float> _step_sizes;


	/* update vars that contain list of all the observations for a given tracking run */
	void update_naive_observations();

	float calculate_step_size();

	// METHODS FOR CALCULATING NEXT COMMANDS

	/* calculate the next tracking command */
	vector<float> calc_next_command();


	/* calculate the next tracking command with a variable step size */
	vector<float> calc_next_command_variable();
};


NaivePlanner::NaivePlanner() : Planner() {
	_observed_bearing.clear();
	_observed_rssi.clear();
	_step_sizes.clear();
}

NaivePlanner::~NaivePlanner() {

}


NaivePlanner::update_naive_observations() {

}

NaivePlanner::calculate_step_size() {

}

NaivePlanner::calc_next_command() {

}

NaivePlanner::calc_next_command_variable() {

}


NaivePlanner::action() {
	// use the most recently calculated bearing and max rssi to update the internal lists
	update_naive_observations();

	// TODO: need some configurability...

	// right now just use the variable step size code
	return calc_next_command_variable();
}