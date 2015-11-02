#ifndef pomdp_h

#define pomdp_h

/* Specific to grid size */
// size 9
//#define GRID_SIZE 9			//size of one side of a cell
//#define NUM_STATES 6561
//#define CENTER_CELL 4
//#define CELL_METERS 11.0		// Size of an individual cell

// size 21
#define GRID_SIZE 21		//size of one side of a cell
#define NUM_STATES 194481
#define CENTER_CELL 10
#define CELL_METERS 11.0		// Size of an individual cell

// size 31
/*
#define GRID_SIZE 31		//size of one side of a cell
#define NUM_STATES 923521
#define CENTER_CELL 15
#define CELL_METERS 11.0		// Size of an individual cell
*/

#define NUM_OBS 37
#define NULL_OBS 36
#define OBS_BINS 3

#define NULL_PROB 0.000001

/* action macros */
/*
 * 0 - n
 * 1 - w
 * 2 - s
 * 3 - e
 * 4 - nw
 * 5 - sw
 * 6 - ne
 * 7 - se
 * 8 - p
 * 9 - r
 */
#define NUM_ACTIONS 10
#define ACTION_N 0
#define ACTION_W 1
#define ACTION_S 2
#define ACTION_E 3
#define ACTION_NW 4
#define ACTION_SW 5
#define ACTION_NE 6
#define ACTION_SE 7
#define ACTION_STOP 8
#define ACTION_ROTATE 9

/* vector is often used */
#include <vector>
using std::vector;
using std::pair;

/* math is often used */
#include <cmath>

class POMDP
{
	public:
		int grid_size;	//size of one side of grid
		int num_states;
		int num_actions;
		POMDP();
};


/***************************************************************************
 * POMDP SETUP
***************************************************************************/
int make_alpha_vectors(vector<vector<double> >& alpha_vectors);
int make_bin_probs(vector<vector<vector<double> > >& bin_probs);
void initialize_pomdp();

/***************************************************************************
 * POMDP Workings
***************************************************************************/
double O(vector<int>& sp, int o);

/***************************************************************************
 * Belief Stuff
***************************************************************************/
int update_position(int new_x, int new_y);
int update_belief(int o);
int initialize_belief();
double compress_belief();
pair<int, int> vehicle_xy();
vector<vector<double> > getb2mat();
int write_belief();
int write_bearing(double bearing);
int write_action(int x, int y, double dnorth, double deast);
pair<int, int> action2diff(int a);
double p_obs(vector<int>& sp, int xvp, int yvp, int o);
double cond_obs_update(vector<int>& sp, int xvp, int yvp, int o);

/***************************************************************************
 * MATH
***************************************************************************/
int ind2state(vector<int>& state, int ind);
vector<int> ind2state(int ind);
int state2ind(vector<int>& state);
int sanitize_obs(double obs);
int reverse_obs(int obs);

/***************************************************************************
 * Action Selection
***************************************************************************/
pair<float, float> get_next_pomdp_action(double &bearing, int &rssi);
pair<float, float> action_naiive();
pair<float, float> action_qmdp();
pair<float, float> action_info_theoretic();

#endif
