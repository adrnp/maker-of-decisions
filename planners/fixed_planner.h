#ifndef _FIXED_PLANNER_H_
#define _FIXED_PLANNER_H_

#include "planner.h"


/**
 * this is a "planner" that just cycles a predefined set of commands.
 *
 * these commands are given by the command file, so need to figure out how
 * to handle that in the initialization function...
 */
class FixedPlanner : public Planner {

public:

	/* constructor */
	FixedPlanner(const char * command_file_name);

	/* desctructor */
	~FixedPlanner();

	/* initialize the planner */
	bool initialize();

	/* return the action */
	vector<float> action();

private:

	// the file containing all the commands
	const char * _command_file_name;

	// the commands themselves
	vector<float> _cmd_north;
	vector<float> _cmd_east;
	vector<float> _cmd_alt;

	// file details
	_num_cmds;

	// index param
	_cmd_index;

};



#endif	/* _FIXED_PLANNER_H_ */