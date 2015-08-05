#include "bearing.h"
#include <inttypes.h>


// helper function
double calc_max_angle(double &angle1, double &angle2);



double get_bearing_max(vector<double> &angles, vector<double> &gains) {

	// NOTE: this does assume that angles and gains are of the same length
	// to my knowledge this is a valid assumption, but should probably be checked

	double max_gain = -100;
	double i_max = 0;
	double angle_max1 = 0;
	double angle_max2 = INT_MAX;

	int len = gains.size();
	for (int i = 0; i < len; i++) {
		
		// ignore uncollected samples
		if (gains[i] == INT_MAX) {
			continue;
		}

		// check conditions in which we want to save information
		if (gains[i] > max_gain) {		// found a new max location
			max_gain = gains[i];
			i_max = i;
			angle_max1 = angles[i];

		} else if (gains[i] == max_gain) {	// tied max location, so want to keep angle
			angle_max2 = angles[i];
		}
	}

	// calculate the bearing given the max information
	double angle_max = angle_max1;
	if (angle_max2 != INT_MAX) {

		// this won't wrap around 360!!!!
		angle_max = (angle_max1 + angle_max2)/2;
	}

	return angle_max;

}


double calc_max_angle(double &angle1, double &angle2) {
	
}