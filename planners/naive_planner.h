#ifndef _NAIVE_PLANNER_H_
#define _NAIVE_PLANNER_H_

#include "planner.h"

class NaivePlanner : public Planner {

public:

	/* constructor */
	NaivePlanner();

	/* desctructor */
	~NaivePlanner();

	/* initialize the planner */
	bool initialize();

	/* return the action */
	vector<float> action();

private:

	// overall information used to make decisions
	vector<double> _observed_bearing;
	vector<int> _observed_rssi;
	vector<float> _step_sizes;

	bool _first_step;


	/* update vars that contain list of all the observations for a given tracking run */
	void update_naive_observations();
	float calculate_step_size();

	// METHODS FOR CALCULATING NEXT COMMANDS

	/* calculate the next tracking command */
	vector<float> calc_next_command(const double &bearing, const double &rssi);


	/* calculate the next tracking command with a variable step size */
	vector<float> calc_next_command_variable(const double &bearing, const double &rssi);
};

#endif	/* _NAIVE_PLANNER_H_ */