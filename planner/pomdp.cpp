#include "pomdp.h"

//using std::vector::resize;
//
//n,w,s,e,nw,sw,ne,se,p,r
/**
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

using std::atan2;
using std::fmod;
using std::abs;

POMDP::POMDP()
{
	grid_size = 21;
	num_states = 21^4;
	num_actions = 8;

	/* create stored_alphas */
	make_alphas(stored_alphas);

	/* create obs_probs */
	make_obs_probs(obs_probs);
}


double T(vector<int>& s_state, int a, vector<int>& sp_state)
{
	int xp = s_state[0];
	int yp = s_state[1];
	double prob = 0.0;

	/* East-West */
	if (a == 1 || a == 4 || a == 5)
	{
		if (xp > 0)
			xp -= 1;
	}
	else if (a == 3 || a == 6 || a == 7)
	{
		if (xp < 20)
			xp += 1;
	}


	/* North-South */
	if (a == 0 || a == 4 || a == 6)
	{
		if (yp < 20)
			yp += 1;
	}
	else if (a == 2 || a == 5 || a == 7)
	{
		if (yp > 0)
			yp -= 1;
	}

	if (xp == sp_state[0] && yp == sp_state[1])
		prob = 1.0;

	return prob;
}


/**
 * Could just pass in model instead of passing in stored_alphas, obs_probs
 */
double O(int a, vector<int>& sp, int o, vector<vector<int> >& stored_alphas, vector<double>& obsProbs)
{
	// I imagine there are 37 total observations
	// numbered from 0 to 36
	// 36 is the null obs 
	if (a != ACTION_ROTATE)
	{
		if (o != NULL_OBS)
			return 1.0;
		else
			return 0.0;
	}

	/* If we get here, we must have rotated */
	int xr = sp[2] - sp[0];
	int yr = sp[3] - sp[1];

	/* handle blind spot stuff */
	if (xr == 0 && yr == 0)
	{
		/* means we should get no observations */
		if (o == NULL_OBS)
			return 1.0;
		else
			return 0.0;
	}

	/* If we get here, we have rotated and are not in a blind spot */
	/* If obs is that we get no bearing, return 0 */
	if (o == NULL_OBS)
		return 0.0;

	/* If we get here, we have rotated and are not in the blind spot */
	/* We have also received some angle measurement */
	int alpha = stored_alphas[xr + 20][yr + 20];
	int obsDifference = abs(alpha - o);
	int min_val, max_val;

	/* TODO: check that this indexing is correct */
	/* this requirs a more thorough test */
	/* See the third trial in test_O in test.cpp */
	/* It passes there, so I feel good about it, but not 100%*/
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
		return obsProbs[NUM_OBS - obsDifference - 1];
	}
	else if (obsDifference > OBS_BINS)
	{
		/* If difference in angles is greater than a few std deviations, */
		/*  we can assume this has zero probability */
		return 0.0;
	}
	else
	{
		/* This is the case that there is some probability for this obs */
		/* This meas our observation difference is small */
		return obsProbs[obsDifference];
	}

	/**
	 * TODO: to make this function work:
	 *  stored_alphas = matrix of angles for every possible delta_x, delta_y combination
	 *  obsBins = Number of std deviations we care about?
	 *  k
	 */
}

/**
 * Stores angles for every state to avoid expensive atan2 calls
 */
// TODO: resize stored_alphas
int make_alphas(vector<vector<int> >& stored_alphas)
{
	int max_rel = 20;	// max distance in any one direction

	/* Resize stored_alphas */
	int mat_size = 2*max_rel + 1;
	int i;
	stored_alphas.resize(mat_size);
	for (i = 0; i < mat_size; i++)
		stored_alphas[i].resize(mat_size);

	/* Compute the angles */
	int xr, yr, alpha;
	double temp;
	for (xr = -max_rel; xr <= max_rel; xr++)
	{
		for (yr = -max_rel; yr <= max_rel; yr++)
		{
			/* Calculate alpha */
			temp = atan2(yr, xr) * (180.0 / M_PI);
			if (temp < 0)
				temp += 360.0;
			temp = fmod(temp, 360.0);
			alpha = ((int)temp) / 10; // m.unitAngle = 10
			
			/* Store alpha */
			stored_alphas[xr + max_rel][yr + max_rel] = alpha;
		}
	}
	return 0;
}

int make_obs_probs(vector<double>& obs_probs)
{
	obs_probs.resize(4);
	obs_probs[0] = 0.383103;
	obs_probs[1] = 0.241843;
	obs_probs[2] = 0.0606257;
	obs_probs[3] = 0.00597982;
	return 0;
}
