#include <stdio.h>
#include <math.h>

#include "policyExecution.h"

double obsFunction(int o, int s[], int a)
{
	//int numberOfObservations = m.observationSizes[1];
	int numberOfObservations = 73;
	double obsProbs[3] = {.4, .2, .1};

	// If you don't rotate
	if (a != 4)
	{
		if (o == (numberOfObservations - 1))
			return 1.0;
		else
			return 0.0;
	}
	
	// Case that we do rotate 
	int xr = s[3] - s[1];
	int yr = s[4] - s[2];


	if (sqrt(pow(xr,2) + pow(yr,2))*CELL_SIZE < ALTITUDE/sqrt(3))
	{
		// This means we get should no observations...
		if (o == (numberOfObservations - 1))
			return 1.0;
		else
			return 0.0;
	}


	// If we get here, we have rotated and are not in blind spot
	// We should get a result - if we the obs is that we don't, return 0
	/*if obsNumber == (numberOfObservations-1)
		return 0
	end*/
	if (o == (numberOfObservations - 1))
	{
		return 0.0;
	}

	// If we have gotten to here, we have rotated, are not in the blind spot,
	//	and have received some angle measurement

	// determine mean from state
	//int alpha = int( fld( mod(atan2(yr, xr)*(180.0/M_PI), 360) , m.unitAngle))
	int alpha = ( (int)( atan2(yr, xr)*(180.0/M_PI) ) ) % 360 / UNIT_ANGLE;

	int minVal;
	int maxVal;
	int obsDifference = abs(alpha - o);
	if (obsDifference > (numberOfObservations - NUM_SIGMAS*SIGMA))
	{
		/* return cycle difference */
		if (o < alpha)
		{
			minVal = o;
			maxVal = alpha;
		}
		else
		{
			minVal = alpha;
			maxVal = o;
		}
		return obsProbs[o - maxVal + minVal];
	}
	else if (obsDifference > SIGMA*NUM_SIGMAS)
	{
		return 0.0;
	}
	else
	{
		return obsProbs[obsDifference];
	}
}
