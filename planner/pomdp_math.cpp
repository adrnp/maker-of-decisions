/**
 * pomdp_math.cpp
 *
 * Includes a lot of the math needed to run or initialize the POMDP.
 *
 * Do I really need to include pomdp.h
 */
#include "pomdp.h"

/**
 * Assumes that we are working in grid with 21 cells per side.
 * This appears to be working.
 *
 * Assumes you pass in vector of 4 states
 */
int ind2state(vector<int>& state, int ind)
{
	int stride = pow(GRID_SIZE, 3);
	int i, rest;
	for (i = 3; i > 0; i--)
	{
		rest = ind % stride;
		state[i] = (ind - rest) / stride;
		ind = rest;
		stride = stride / GRID_SIZE;
	}
	state[0] = ind;
	return 0;
}

vector<int> ind2state(int ind)
{
	vector<int> state(4);
	ind2state(state, ind);
	return state;
}


int state2ind(vector<int>& state)
{
	int sum, i, j, size_prod;
	sum = state[0];

	for (i = 3; i > 0; i--)
	{
		size_prod = 1;
		for (j = 0; j < i; j++)
		{
			size_prod *= GRID_SIZE;
		}
		sum += state[i] * size_prod;
	}
	return sum;
}

/* make obs between 0 and 35 */
/* close enough ? */
int sanitize_obs(double obs)
{
	int ret_obs = round(obs / 10.0);
	if (ret_obs == 36)
		ret_obs = 35;
	return ret_obs;

	/*
	int ret_obs = reverse_obs((int)obs);
	ret_obs = (int) (ret_obs / 10.0);
	if (ret_obs == 36)
		ret_obs = 35;
	return ret_obs;
	*/
}


/**
 * get_bearings return east of north
 * observation function expects north of west
 *
 * This assumes obs from 0 to 359 (not 0 - 35)
 * I'm pretty sure this works
 */
int reverse_obs(int obs)
{
	int f = 90 - obs;
	if (f < 0)
		f += 360;
	return f;
}
