/* header file for tracker.cpp.
 * tracker contains all the functions required for
 * generating tracking commands following different techniques.
 */


#ifndef TRACKER_H_
#define TRACKER_H_

#include <vector>

// HELPER FUNCTIONS

/* update global vars that contain list of all the observations for a given tracking run */
void update_observations(double &bearing, int &rssi);

float calculate_step_size();


// METHODS FOR CALCULATING NEXT COMMANDS

/* calculate the next tracking command */
std::vector<float> calc_next_command(double &bearing, int &rssi);


/* calculate the next tracking command with a variable step size */
std::vector<float> calc_next_command_variable(double &bearing, int &rssi);


#endif /* TRACKER_H_ */
