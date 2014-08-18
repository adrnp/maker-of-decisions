#include <stdio.h>
#include <math.h>

#include "constants.h"
#include "policyExecution.h"

int initBelief(double *belief, int xStart, int yStart)
{
	int i;
	int s[] = {[0 ... NUM_STATE_VARS-1] = 0};
	int stateSizes[] = {[0 ... NUM_STATE_VARS-1] = 5};

	for (i = 0; i < VECTOR_LENGTH; i++)
	{
		ind2state(s, stateSizes, i);
		if ( (s[0] == xStart) && (s[1] == yStart) )
		{
			belief[i] = 1.0/(GRID_SIZE*GRID_SIZE);
		}
	}
}

int updateBelief(double *b, double *bprime, int a, int o)
{
	
	int s_prime[NUM_STATE_VARS];
	int s[NUM_STATE_VARS];

	int s_prime_index;
	int s_index;

	int stateSizes[] = {[0 ... NUM_STATE_VARS-1] = 5};

	double oProb;
	int neighbors[5][4];
	int numberOfNeighbors;

	int i;	// counter variable 
	int j;	// counter variable 
	double sum;

	for (s_prime_index = 0; s_prime_index < VECTOR_LENGTH; s_prime_index++)
	{
		/* This is a possible s' */
		ind2state(s_prime, stateSizes, s_prime_index);
		oProb = obsFunction(o, s_prime, a);

		numberOfNeighbors = getNeighbors(neighbors, s_prime);
		sum = 0.0;

		for (i = 0; i < numberOfNeighbors; i++)
		{
			for (j = 0; j < NUM_STATE_VARS; j++)
			{
				s[j] = neighbors[i][j];
			}
			s_index = state2ind(stateSizes, s);
			sum += transitionFunction(s_prime, s, a) * b[s_index];
		}
		bprime[s_prime_index] = sum * oProb;
	}
	return 4;
}
