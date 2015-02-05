#include <stdio.h>
#include <math.h>

#include "policyExecution.h"

// In the julia version:
// The input is 1-indexed, here it is 0-indexed
// The output is 0-indexed, here it is 0-indexed
// I THINK THIS WORKS NOW
int ind2state(int state[], int dims[], int ind)
{
	int ndims = NUM_STATE_VARS;
	int stride = dims[0];
	int i;							// counter
	int rest;
	ind += 1;

	for (i = 1; i < (NUM_STATE_VARS-1); i++)
	{
		stride *= dims[i];
	}
	for (i = (NUM_STATE_VARS-2); i >= 0; i--)
	{
		rest = ((ind-1) % stride) + 1;
		state[i+1] = (int)( (ind - rest) / stride);
		ind = rest;
		stride = (int) (stride / dims[i]);
	}
	state[0] = ind - 1;
	return 0;
}


int state2ind(int stateSizes[], int s[])
{
	int sum = s[0];
	int product;

	/* Generic counter variables */
	int i;
	int j;
	for (i = NUM_STATE_VARS-1; i > 1; i--)
	{
		product = 1;
		for (j = 0; j < i; j++)
		{
			product *= stateSizes[j];
		}
		sum += s[i] * product;
	}
	return sum;
}

double dot_product(double v[], double u[], int n)
{
	double result = 0.0;
	int i;
	for (i = 0; i<n; i++)
	{
		result += v[i]*u[i];
	}
	return result;
}
