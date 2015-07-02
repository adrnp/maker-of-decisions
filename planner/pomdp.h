#ifndef pomdp_h
#define pomdp_h

#define NUM_OBS 37
#define NULL_OBS 36
#define OBS_BINS 3

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
#define ACTION_N 0
#define ACTION_W 1
#define ACTION_S 2
#define ACTION_E 3
#define ACTION_NW 4
#define ACTION_SW 5
#define ACTION_NE 6
#define ACTION_SE 7
#define ACTION_STOP = 8
#define ACTION_ROTATE 9

/* vector is often used */
#include <vector>
using std::vector;

/* math is often used */
#include <cmath>

class POMDP
{
	public:
		int grid_size;	//size of one side of grid
		int num_states;
		int num_actions;
		vector<vector<int> > stored_alphas;	//(36, vector<int>(39));
		vector<double> obs_probs;
		POMDP();
};


/***************************************************************************
 * POMDP SETUP
***************************************************************************/
int make_alphas(vector<vector<int> >& stored_alphas);
int make_obs_probs(vector<double>& obs_probs);

/***************************************************************************
 * POMDP Workings
***************************************************************************/
double T(vector<int>& s_state, int a, vector<int>& sp_state);
double O(int a, vector<int>& sp, int o, vector<vector<int> >& stored_alphas, vector<double>& obsProbs);

/***************************************************************************
 * MATH
***************************************************************************/
int ind2state(vector<int>& state, int ind);
vector<int> ind2state(int ind);
int state2ind(vector<int>& state);

#endif
