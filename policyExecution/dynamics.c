#include <stdio.h>
#include <math.h>

#include "policyExecution.h"

double transitionFunction(int s_prime[], int s[], int a)
{
	int xAdd = 0;
	int yAdd = 0;

	//actions_4 = ["n","w","s","e","r","p"]
	/* Return 0 if jammer location is not the same */
	if ( (s_prime[3] != s[3]) && (s_prime[4] != s[4]) )
		return 0.0;

	// north
	if ((0 == a) && (s[1] < (GRID_SIZE-1)) )
		yAdd = 1;
	// west
	else if ((1 == a) && (s[0] > 0) )
		xAdd = -1;
	// south
	else if ((2 == a) && (s[1] > 0) )
		yAdd = -1;
	// east
	else if ((3 == a) && (s[0] < (GRID_SIZE-1)) )
		xAdd = 1;

	if ( ((s[0]+xAdd) == s_prime[0]) && ((s[1]+yAdd) == s_prime[1]) && (s[2] == s_prime[2]) && (s[3] == s_prime[3]) )
		return 1.0;
	else
		return 0.0;
}

int getNeighbors(int neighbors[][4], int s_prime[])
{
	int neighborIndex = 1;

	/* First neighbor is just the value itself */
	neighbors[0][0] = s_prime[0];
	neighbors[0][1] = s_prime[1];
	neighbors[0][2] = s_prime[2];
	neighbors[0][3] = s_prime[3];

	/* Second neighbor is one grid cell east */
	if ( (s_prime[0] + 1) < GRID_SIZE )
	{
		neighbors[neighborIndex][0] = s_prime[0] + 1;
		neighbors[neighborIndex][1] = s_prime[1];
		neighbors[neighborIndex][2] = s_prime[2];
		neighbors[neighborIndex][3] = s_prime[3];
		neighborIndex++;
	}

	/* Third neighbor is one grid cell west */
	if ( (s_prime[0] - 1) >= 0 )
	{
		neighbors[neighborIndex][0] = s_prime[0] - 1;
		neighbors[neighborIndex][1] = s_prime[1];
		neighbors[neighborIndex][2] = s_prime[2];
		neighbors[neighborIndex][3] = s_prime[3];
		neighborIndex++;
	}

	/* Fourth neighbor is grid cell north */
	if ( (s_prime[1] + 1) < GRID_SIZE )
	{
		neighbors[neighborIndex][0] = s_prime[0];
		neighbors[neighborIndex][1] = s_prime[1] + 1;
		neighbors[neighborIndex][2] = s_prime[2];
		neighbors[neighborIndex][3] = s_prime[3];
		neighborIndex++;
	}

	/* Fifth neighbor is one grid cell south */
	if ( (s_prime[1] - 1) >= 0 )
	{
		neighbors[neighborIndex][0] = s_prime[0];
		neighbors[neighborIndex][1] = s_prime[1] - 1;
		neighbors[neighborIndex][2] = s_prime[2];
		neighbors[neighborIndex][3] = s_prime[3];
		neighborIndex++;
	}
	return neighborIndex;
}
