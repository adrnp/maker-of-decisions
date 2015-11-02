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
using std::cout;
using std::endl;
//using std::ofstream;
std::ofstream pomdpfile;

/* Required for observation function */
vector< vector<vector<double> > > bin_probs;

vector<double> b (NUM_STATES);			// belief vector (4d)
vector<double> btemp (NUM_STATES);		// temporary belief vector (4d)
vector<vector<double> > b2mat;			// jammer belief array (2d)
bool start = true;						// do we need to start?
bool success = false;					// have we found it? max(b) > .9
vector<vector<double> > alpha_vectors;	// alpha vectors

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
			if (si != sni)
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
	double maxb = compress_belief();
	if (maxb > 0.5)
	{
		success = true;
	}
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
double compress_belief()
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

	/* Determine the largest one... */
	double max_b2mat = 0.0;
	for (xj = 0; xj < GRID_SIZE; xj++)
	{
		for (yj = 0; yj < GRID_SIZE; yj++)
		{
			if (b2mat[xj][yj] > max_b2mat)
				max_b2mat = b2mat[xj][yj];
		}
	}
	return max_b2mat;
}

void initialize_pomdp()
{
	//initialize belief vectors
	initialize_belief();
	make_bin_probs(bin_probs);
	make_alpha_vectors(alpha_vectors);
	
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
	pair<int, int> d_pair;
	float delta_north, delta_east;
	int belief_crash;

	if (start)
		initialize_pomdp();

	/* Sanitize observation, update belief */
	int obs = sanitize_obs(bearing); 
	write_bearing(bearing);
	belief_crash = update_belief(obs);
	pomdpfile << "update complete" << endl;
	if (belief_crash == 1)
	{
		pomdpfile << "BELIEF CRASH" << endl;
		return pair<float,float>(1000.0,1000.0);
	}
	write_belief();
	if (success)
	{
		pomdpfile << "JAMMER FOUND" << endl;
		return pair<float,float>(1000.0,1000.0);
	}

	//d_pair = action_naiive();
	pomdpfile << "before action select" << endl;
	//d_pair = action_qmdp();
	d_pair = action_info_theoretic();
	pomdpfile << "after action select" << endl;

	return d_pair;
}


int write_bearing(double bearing)
{
	int obs = sanitize_obs(bearing);
	pomdpfile << std::fixed;
	pomdpfile << std::setprecision(0);
	pomdpfile << "bearing = " << bearing <<  ", o = " << obs << endl;
	return 0;
}

int write_belief()
{
	int x, y;
	pomdpfile << std::setprecision(3);
	for (y = (GRID_SIZE-1); y >= 0; y--)
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
pair<float, float> action_naiive()
{
	// pick the max x,y from b2mat
	double max_belief, temp;
	int x, y;
	int max_x = CENTER_CELL;
	int max_y = CENTER_CELL;
	double delta_north, delta_east;
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

	// ok, handle this shit
	delta_north = CELL_METERS * (max_y - vehicular_y);
	delta_east = CELL_METERS * (max_x - vehicular_x);
	write_action(max_x, max_y, delta_north, delta_east);

	// Assume we go there and update to reflect this
	update_position(max_x, max_y);
	return pair<float, float>(delta_north, delta_east);
}


/**
 * returns cell to travel to
 * TODO: need to break if it just keeps wandering forever
 */
pair<float, float> action_qmdp()
{
	// multiply belief by each alpha_vector
	int a, s, best_a, dx, dy;
	double utility, max_utility;
	pair<int, int> diff_xy;
	best_a = 0;
	dx = 0;
	dy = 0;

	while ( (best_a != ACTION_ROTATE) && (best_a != ACTION_STOP) )
	{
		max_utility = -99999999;
		best_a = 0;
		for (a = 0; a < NUM_ACTIONS; a++)
		{
			utility = 0.0;
			for (s = 0; s < NUM_STATES; s++)
			{
				utility += b[s] * alpha_vectors[a][s];
			}
			if (utility > max_utility)
			{
				max_utility = utility;
				best_a = a;
			}
		}

		// convert this to a distance we must travel
		diff_xy = action2diff(best_a);
		update_position(diff_xy.first + vehicular_x, diff_xy.second + vehicular_y);
		dx += diff_xy.first;
		dy += diff_xy.second;
	}
	double delta_north = CELL_METERS * dy;
	double delta_east = CELL_METERS * dx;

	write_action(vehicular_x, vehicular_y, delta_north, delta_east);

	// TODO: If it is stop, end
	if (best_a == ACTION_STOP)
	{
		//let adrien know...
	}

	if ((delta_north == 0.0) && (delta_east == 0.0))
	{
		delta_east = 1.0;
	}

	return pair<float, float>(delta_north, delta_east);
}

pair<float, float> action_info_theoretic()
{
	int xvp, yvp, o, best_x, best_y;
	double obs_entropy, cond_obs_entropy, p_o, temp_val;
	double best_val = -9999999;
	vector<int> sp(4);
	double delta_north, delta_east;
	for (xvp = 0; xvp < GRID_SIZE; xvp++)
	{
		for (yvp = 0; yvp < GRID_SIZE; yvp++)
		{
			obs_entropy = 0.0;
			cond_obs_entropy = 0.0;
			for (o = 0; o < NUM_OBS; o++)
			{
				p_o = p_obs(sp, xvp, yvp, o);
				if (p_o > 0.0)
					obs_entropy -= p_o * log(p_o);
				cond_obs_entropy -= cond_obs_update(sp, xvp, yvp, o);
			}
			temp_val = obs_entropy - cond_obs_entropy;
			if (temp_val > best_val)
			{
				best_val = temp_val;
				best_x = xvp;
				best_y = yvp;
			}
		}

	}
	delta_north = CELL_METERS * (best_y - vehicular_y);
	delta_east = CELL_METERS * (best_x - vehicular_x);
	write_action(best_x, best_y, delta_north, delta_east);

	// Assume we go there and update to reflect this
	update_position(best_x, best_y);

	return pair<float, float>(delta_north, delta_east);
}


vector<vector<double> > getb2mat()
{
	return b2mat;
}


/**
 * Only called if we've rotated
 * 37 total observations, numbered 0 to 36 (36 is null obs)
 * TODO: should we include blind spot stuff?
 */
double O(vector<int>& sp, int o)
{
	/* We expect no blind spots here. */
	if (o == NULL_OBS)
		return 0.0;

	// Here, we've rotated and have not received the null observation
	// Determine xr, yr of jammer relative to you
	int xr = sp[2] - sp[0];
	int yr = sp[3] - sp[1];

	return bin_probs[xr+GRID_SIZE-1][yr+GRID_SIZE-1][o];
}


double p_obs(vector<int>& sp, int xvp, int yvp, int o)
{
	double prob = 0.0;
	sp[0] = xvp;
	sp[1] = yvp;
	int xj, yj;
	for (xj = 0; xj < GRID_SIZE; xj ++)
	{
		for (yj = 0; yj < GRID_SIZE; yj++)
		{
			sp[2] = xj;
			sp[3] = yj;
			prob += b2mat[xj][yj] * O(sp, o);
		}
	}
	return prob;
}

double cond_obs_update(vector<int>& sp, int xvp, int yvp, int o)
{
	double temp = 0.0;
	double pobs;
	int xj, yj;
	sp[0] = xvp;
	sp[1] = yvp;
	for (xj = 0; xj < GRID_SIZE; xj++)
	{
		for (yj = 0; yj < GRID_SIZE; yj++)
		{
			sp[2] = xj;
			sp[3] = yj;
			pobs = O(sp, o);
			if (pobs > 0.0)
				temp += b2mat[xj][yj] * pobs * log(pobs);
		}
	}
	return temp;
}
