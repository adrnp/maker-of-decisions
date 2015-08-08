#include <vector>

/* function to get the next (N,E) command from the pomdp logic, given an observation */
std::pair<float, float> get_next_pomdp_action(double &bearing, int &rssi);

/* update the known observation information */
void update_observations(double &bearing, int &rssi);

/* this does the actual pomdp stuff, using the most recent observations, update the commanded grid cell pair */
void update_next_commanded_cell();