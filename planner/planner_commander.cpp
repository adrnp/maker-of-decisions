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

#include "planner_commander.h"

using std::string;
using std::vector;
using std::pair;
using namespace std;

int test_num = 0;

#define GRID_ROWS 20
#define GRID_COLS 20

#define CELL_WIDTH 10.0

// variables to keep track of state information
uint8_t grid[GRID_ROWS][GRID_COLS] = {{0}};			// grid of the world, not sure if needed???

vector<pair<int, int>> cells_visited;			// vector of pairs defining which grid cells have been visited
vector<double> observed_bearings;				// vector of bearing observations from each location visited
vector<int> observed_rssi;						// vector of max rssi observations from each location visited

pair<int, int> prev_commanded_grid_cell;			// the index of the previously commanded grid cell
pair<int, int> commanded_grid_cell;				// the index of the commanded grid cell

// TODO: put additional global variables here needed to keep track of any styate information desired


pair<float, float> get_next_pomdp_action(double &bearing, int &rssi) {

	// update the observation states
	update_observations(bearing, rssi);

	// update which is the next grid cell we want to go to
	update_next_commanded_cell();

	// convert the grid cell changes to actual distances
	float d_south = (float)(commanded_grid_cell.first - prev_commanded_grid_cell.first) * CELL_WIDTH;		// note rows increase SOUTH!!!
	float d_east = (float)(commanded_grid_cell.second - prev_commanded_grid_cell.second) * CELL_WIDTH;		// cols increase EAST

	// return the pair that contains the delta information
	return pair<float, float>(-d_south, d_east);
}


void update_observations(double &bearing, int &rssi) {

	// TODO: add any additional update information that would be required

	// note that we have just visited the commanded grid cell
	cells_visited.push_back(commanded_grid_cell);

	// add the observed bearing and rssi to the list
	observed_bearings.push_back(bearing);
	observed_rssi.push_back(rssi);

	return;
}


void update_next_commanded_cell() {

	// TODO: do the actual pomdp stuff given the observations information

	// update the previous commanded grid cell index
	prev_commanded_grid_cell = commanded_grid_cell;

	// for now, just increase the indices
	// increase the row, until end of a row, and then increase the column
	if (commanded_grid_cell.first++ >=  GRID_ROWS) {
		commanded_grid_cell.first = 0;

		if (commanded_grid_cell.second++ >= GRID_COLS) {
			commanded_grid_cell.second = 0;
		}

	}

}
