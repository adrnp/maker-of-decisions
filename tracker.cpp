// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <vector>
#include <sys/time.h>


// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#
#ifdef __linux
#include <sys/ioctl.h>
#endif

#include "common.h"
#include "tracker.h"

using std::vector;
using namespace std;


// variable step size calculation params
#define BEARING_TOL 10.0			// desired tolerance in degrees for 2 bearing measurements to be considered the same
#define STEP_INCREASE_FACTOR 2.0	// factor by which step size increases if along the same path as before
#define STEP_SMALL 10.0
#define STEP_LARGE 50.0


vector<double> observedBearing;
vector<int> observedRssi;
vector<float> stepSizes;

// just to know if this is the first step
bool firstStep = true;

void update_observations(double &bearing, int &rssi) {

	// simply add to the list of observations for these
	observedBearing.push_back(bearing);
	observedRssi.push_back(rssi);

}


float calculate_step_size() {

	// figure out how many observations have been made
	int numObs = observedBearing.size();

	float prevStepSize = stepSizes[stepSizes.size() - 1];
	float nextStepSize = prevStepSize;

	float difference = abs(observedBearing[numObs-1] - observedBearing[numObs-2]); 
	printf("[NAIVE] difference is %f\n", difference);
	if (difference < BEARING_TOL) {
		printf("[NAIVE] step sizes within tolerance\n");
		// double the step size
		nextStepSize *= STEP_INCREASE_FACTOR;
	} else {
		printf("[NAIVE] step size not within tol\n");
		// reset back to the smallest step size
		nextStepSize = STEP_SMALL;
	}

	return nextStepSize;
}



vector<float> calc_next_command(double &bearing, int &rssi) {
	// bearing is degrees from 0 to 359

	// update the observation information
	update_observations(bearing, rssi);

	if (verbose) {
		printf("calculating the next command with input (%f, %i)\n", bearing, rssi);
	}
	
	// for debug purposes, force a specific bearing and rssi
	double bear = bearing;

	if (bear >= 360.0) {
		bear = bear - 360.0;
	}

	int rs = 20;

	// commands are a vector of [north, south]
	vector<float> commands;

	float k = 0.5; // units: m / dB

	//float north = k * (double) rssi * cos(bearing * M_PI/180.0);
	//float east = k * ( double) rssi * sin(bearing * M_PI/180.0);

	float north = k * (double) rs * cos(bear * M_PI/180.0);
	float east = k * ( double) rs * sin(bear * M_PI/180.0);

	commands.push_back(north);
	commands.push_back(east);

	return commands;
}



vector<float> calc_next_command_variable(double &bearing, int &rssi) {
	// update the observation information
	update_observations(bearing, rssi);

	float step = STEP_SMALL;
	if (firstStep) {
		//step = false;
	} else {
		step = calculate_step_size();
	}

	stepSizes.push_back(step);

	if (verbose) {
		printf("calculating the next variable size command with input (%f, %i)\n", bearing, rssi);
	}

	// commands are a vector of [north, south]
	vector<float> commands;

	float north = step * cos(bearing * M_PI/180.0);
	float east = step * sin(bearing * M_PI/180.0);

	commands.push_back(north);
	commands.push_back(east);

	return commands;
}
