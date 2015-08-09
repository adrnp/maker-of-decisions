/**
 * belief.cpp
 *
 * Handles the belief updating
 */

#include "pomdp.h"

/* For writing to file */
#include <iostream>
#include <iomanip>
#include <fstream>
using std::endl;
//using std::ofstream;
std::ofstream pomdpfile;

/* Required for observation function */
vector<double> obs_probs;
vector< vector<int> > stored_alphas;

vector<double> b (NUM_STATES);		// belief vector (4d)
vector<double> btemp (NUM_STATES);	// temporary belief vector (4d)
vector<vector<double> > b2mat;		// jammer belief array (2d)
bool start = true;					// do we need to start?

int vehicular_x;	// vehicle's east-west grid cell
int vehicular_y;	// vehicle's north-south grid cell


/**
 * Updates belief vector due to vehicle motion only
 * Assumes vehicular_x,y are old ocations
 *  new_x,y are new locations
 *
 * Also updates vehicular_x, vehicular_y
 */
int update_position(int new_x, int new_y)
{
	vector<int> s(4);
	vector<int> snew(4);
	s[0] = vehicular_x;
	s[1] = vehicular_y;
	snew[0] = new_x;
	snew[1] = new_y;
	int xj, yj, si, sni;
	for (xj = 0; xj < GRID_SIZE; xj++)
	{
		for (yj = 0; yj < GRID_SIZE; yj++)
		{
			s[2] = xj;
			s[3] = yj;
			snew[2] = xj;
			snew[3] = yj;
			si = state2ind(s);
			sni = state2ind(snew);
			b[sni] = b[si];
			b[si] = 0.0;
		}
	}
	vehicular_x = new_x;
	vehicular_y = new_y;
}


/**
 * Updates bprime
 *
 * Returns 1 if belief crash, 0 otherwise
 */
int update_belief(int o)
{
	int num_states = NUM_STATES;
	int s_index, sp_index;

	double inner_sum, oprob, outer_sum;
	outer_sum = 0.0;
	vector<int> sp(4);

	/* loop over all states */
	for (sp_index = 0; sp_index < num_states; sp_index++)
	{
		ind2state(sp, sp_index);
		oprob = O(sp, o);
		inner_sum = oprob * b[sp_index];
		btemp[sp_index] = inner_sum;
		outer_sum += inner_sum;
	}

	/* Normalize, but first check for belief crash */
	int belief_crash = 0;
	if (outer_sum == 0.0)
		belief_crash = 1;
	else
	{
		for (sp_index = 0; sp_index < num_states; sp_index++)
		{
			b[sp_index] = btemp[sp_index] / outer_sum;
		}
	}
	compress_belief();
	return belief_crash;
}


/**
 * modify b
 */
int initialize_belief()
{
	/* Initialize the 1d belief vector */
	int i;
	vector<int> s(4);
	double prob = 1.0 / pow(GRID_SIZE,2);
	
	for (i = 0; i < NUM_STATES; i++)
	{
		ind2state(s, i); //error in this first line??
		if ( (s[0] == CENTER_CELL) && (s[1] == CENTER_CELL) )
			b[i] = prob;
		else
			b[i] = 0.0;
	}
	
	/* Initialize 2d matrix */
	b2mat.resize(GRID_SIZE);
	int j;
	for (i = 0; i < GRID_SIZE; i++)
	{
		b2mat[i].resize(GRID_SIZE);
		for (j = 0; j < GRID_SIZE; j++)
		{
			b2mat[i][j] = 0.0;
		}
	}
	compress_belief();
}


// assumes all beliefs are zero except where we know we are
pair<int, int> vehicle_xy()
{
	// find first non-zero belief
	int i;
	int x;
	int y;
	vector<int> s(4);
	for (i = 0; i < NUM_STATES; i++)
	{
		// Determine vehicle location
		if (b[i] > 0.0)
		{
			ind2state(s, i);
			x = s[0];
			y = s[1];
		}
	}
	return pair<int,int>(x,y);
}


/**
 * Convert 4-d to 2-d
 * basically asks, where do we think jammer could be?
 */
void compress_belief()
{
	// basically need to sum across the 3rd and 4th cols
	int i, xj, yj;

	/* Clear up b2mat */
	for (xj = 0; xj < GRID_SIZE; xj++)
	{
		for (yj = 0; yj < GRID_SIZE; yj++)
		{
			b2mat[xj][yj] = 0.0;
		}
	}

	vector<int> s(4);
	for (i = 0; i < NUM_STATES; i++)
	{
		ind2state(s, i);
		xj = s[2];
		yj = s[3];

		// we have xv, yv, xj, yj
		// for all xj, yj, add the stuff up
		b2mat[xj][yj] += b[i];
	}
}

void initialize_pomdp()
{
	//initialize belief vectors
	initialize_belief();
	make_obs_probs(obs_probs);
	make_alphas(stored_alphas);
	
	// initialize where we think we are (center of grid)
	vehicular_x = CENTER_CELL;
	vehicular_y = CENTER_CELL;

	// open output file
	pomdpfile.open("pomdpfile.txt");
	
	// let us know that we are not in the startup phase
	start = false;
}

/**
 * Updates belief
 * Determines where to go next
 * Updates vehicular_x, vehicular_y to where we going
 *
 * Will only be called after first observation is made
 */
pair<float, float> get_next_pomdp_action(double &bearing, int &rssi)
{
	pair<int, int> xy_pair;
	float delta_north, delta_east;
	int belief_crash;

	if (start)
		initialize_pomdp();

	/* Sanitize observation, update belief */
	int obs = sanitize_obs(bearing); 
	belief_crash = update_belief(obs);
	if (belief_crash == 1)
	{
		pomdpfile << "BELIEF CRASH" << endl;
		return pair<float,float>(1000.0,1000.0);
	}
	write_belief(bearing);

	// Then, determine x, y to go to
	xy_pair = next_action_naiive();
	
	// Determine delta values to get there
	delta_north = CELL_METERS * (xy_pair.second - vehicular_y);
	delta_east = CELL_METERS * (xy_pair.first - vehicular_x);
	write_action(xy_pair.first, xy_pair.second, delta_north, delta_east);

	// Assume we go there and update to reflect this
	update_position(xy_pair.first, xy_pair.second);

	return pair<float, float>(delta_north, delta_east);
}


int write_belief(double bearing)
{
	int obs = sanitize_obs(bearing);
	int x, y;

	pomdpfile << std::fixed;
	pomdpfile << std::setprecision(0);
	pomdpfile << "bearing = " << bearing <<  ", o = " << obs << endl;
	pomdpfile << std::setprecision(3);
	for (y = GRID_SIZE; y >= 0; y--)
	{
		for (x = 0; x < (GRID_SIZE-1); x++)
		{
			pomdpfile << b2mat[x][y] << ",";
		}
		pomdpfile << b2mat[GRID_SIZE-1][y] << endl;
	}
	return 0;
}

int write_action(int x, int y, double dnorth, double deast)
{
	pomdpfile << std::setprecision(0);
	pomdpfile << "New grid cell commanded <x,y> = " << x << "," << y << endl;
	pomdpfile << "Commanded movement <dnorth, deast>(meters) = ";
	pomdpfile << dnorth << "," << deast << endl << endl;
}


/**
 * Returns the <x,y> cell you should move to
 */
pair<int, int> next_action_naiive()
{
	// pick the max x,y from b2mat
	double max_belief, temp;
	int x, y;
	int max_x = CENTER_CELL;
	int max_y = CENTER_CELL;
	for (x = 0; x < GRID_SIZE; x++)
	{
		for (y = 0; y < GRID_SIZE; y++)
		{
			temp = b2mat[x][y];
			if (temp > max_belief)
			{
				max_belief = temp;
				max_x = x;
				max_y = y;
			}
		}
	}
	return pair<int,int>(max_x, max_y);
}


vector<vector<double> > getb2mat()
{
	return b2mat;
}


/**
 * Only called if we've rotated
 * 37 total observations, numbered 0 to 36 (36 is null obs)
 * TODO: should we include bling spot stuff?
 */
double O(vector<int>& sp, int o)
{
	int xr = sp[2] - sp[0];
	int yr = sp[3] - sp[1];

	/* handle blind spot stuff */
	if (xr == 0 && yr == 0)
	{
		/* means we should get no observations */
		if (o == NULL_OBS)
			return 1.0;
		else
			return NULL_PROB;
	}

	/* If we get here, we have rotated and are not in a blind spot */
	/* If obs is that we get no bearing, return 0 */
	if (o == NULL_OBS)
		return NULL_PROB;

	/* If we get here, we have rotated and are not in the blind spot */
	/* We have also received some angle measurement */
	int alpha = stored_alphas[xr + GRID_SIZE-1][yr + GRID_SIZE-1];
	int obsDifference = std::abs(alpha - o);
	int min_val, max_val;

	if (obsDifference > (NUM_OBS - 2 - OBS_BINS))
	{
		/*
		 * This is the case that the difference in angles is not very large,
		 *  but seems like it because o is 0 and alpha is 71
		 * With 72 observations, it seems like the obsDifference is 71
		 * But really, it is 1.
		 * This is related to cycle difference.
		*/
		if (o < alpha)
		{
			min_val = o;
			max_val = alpha;
		}
		else
		{
			min_val = alpha;
			max_val = o;
		}
		return obs_probs[NUM_OBS - obsDifference - 1];
	}
	else if (obsDifference > OBS_BINS)
	{
		/* If difference in angles is greater than a few std deviations, */
		/*  we can assume this has zero probability */
		return NULL_PROB;
	}
	else
	{
		/* This is the case that there is some probability for this obs */
		/* This meas our observation difference is small */
		return obs_probs[obsDifference];
	}
}
