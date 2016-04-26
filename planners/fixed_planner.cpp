
#include "fixed_planner.h"


FixedPlanner(const char * command_file_name) : Planner(),
_command_file_name(command_file_name)
_num_cmds(0),
_cmd_index(0)
{
	_cmd_north.clear();
	_cmd_east.clear();
	_cmd_alt.clear();
}


~FixedPlanner() {

}


bool FixedPlanner::initialize() {

	// load up all the commands from the file
	string file_name(_command_file_name);
	ifstream cmd_file (file_name);

	if (!cmd_file.is_open()) {
		// means there is an error in loading the file
		return false;
	}

	// make sure vectors are clean
	cmd_north.clear();
	cmd_east.clear();
	cmd_alt.clear();

	float cmdN;
	float cmdE;
	float cmdA;
	char comma;
	while (cmd_file >> cmdN >> comma >> cmdE >> comma >> cmdA) {
		_cmd_north.push_back(cmdN);
		_cmd_east.push_back(cmdE);
		_cmd_alt.push_back(cmdA);

		printf("[FIXED PLANNER] North command: %f\n", cmdN);
		printf("[FIXED PLANNER] East cmd: %f\n", cmdE);
		printf("[FIXED PLANNER] Alt cmd: %f\n", cmdA);
	}

	printf("[FIXED PLANNER] num commands read: %d\n", num_cmds);

	_num_cmds = _cmd_north.size();
	return true;

}


vector<float> FixedPlanner::action() {

	// cycle the cmds (ids should go from 0 -> 3)
	if (_cmd_index >= _num_cmds) {
		_cmd_index = 0;
	}

	printf("[FIXED PLANNER] sending move command with index: %d\n", _cmd_index);

	// extract the next north and east commands
	float nextNorth = _cmd_north[_cmd_index];
	float nextEast = _cmd_east[_cmd_index];
	float nextAlt = _cmd_alt[_cmd_index];

	printf("[FIXED PLANNER] sending command %i: N %f\tE %f\tA %f\n", _cmd_index, nextNorth, nextEast, nextAlt);

	_cmd_index++;

	vector<float> next_command;
	next_command.clear();
	next_command.push_back(nextNorth);
	next_command.push_back(nextEast);
	next_command.push_back(nextAlt);

	// return the next command to be used
	return next_command;
}
