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
 */
int ind2state(vector<int>& state, int ind)
{
	int stride = 9261; // 21^3
	int i, rest;
	for (i = 3; i > 0; i--)
	{
		rest = ind % stride;
		state[i] = (ind - rest) / stride;
		ind = rest;
		stride = stride / 21;
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
			size_prod *= 21;
		}
		sum += state[i] * size_prod;
	}
	return sum;
}
