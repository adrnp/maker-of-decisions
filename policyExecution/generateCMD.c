#include "policyExecution.h"

/* Return a command structure */
int generateCommand(double *belief)
{
	/* First, get observation */
	int obs;

	/* Update the belief */
	updateBelief(belief, bprime, bestAction, obs);

	/* Select action based on new belief */
	bestAction = getAction(belief, alphaVectors);
	//actions_4 = ["n","w","s","e","r","p"]

	float currentHeading;
	float currentAltitude;
	double north = 0.0;
	double east = 0.0;
	float yaw_angle;

	if (bestAction == 0)
	{
		north = CELL_SIZE;
		cmd_type = 
	}
	else if (bestAction == 1)
	{
		east = -CELL_SIZE;
	}
	else if (bestAction == 2)
	{
		north = -CELL_SIZE;
	}
	else if (bestAction == 3)
	{
		east = CELL_SIZE;
	}
	else if (bestAction == 4)
	{
		/* Here, we are rotating */
		yaw_angle
	}
	else
	{
	}
	
	mavlink_tracking_cmd_t command;

	command.north = north;
	command.east = east;
	command.yaw_angle = yaw_angle;
	command.altitude = currentAltitude;
	command.cmd_id = cmd_id;
	command.cmd_type = cmd_type;
}
