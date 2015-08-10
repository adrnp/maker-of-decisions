#include "pomdp.h"

#include <fstream>

// I don't think I really use this
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


/**
 * Stores angles for every state to avoid expensive atan2 calls
 */
int make_alphas(vector<vector<int> >& stored_alphas)
{

	/* Resize stored_alphas */
	int i;
	int max_rel = GRID_SIZE - 1;
	int mat_size = 2*max_rel + 1;
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
			temp = std::atan2(yr, xr) * (180.0 / M_PI);
			if (temp < 0)
				temp += 360.0;
			temp = std::fmod(temp, 360.0);
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

int make_alpha_vectors(vector<vector<double> >& alpha_vectors)
{
	alpha_vectors.resize(10);
	int a, s;
	double val;
	std::ifstream alphafile;
	alphafile.open("alpha_vectors.bin");
	for (a = 0; a < 10; a++)
	{
		alpha_vectors[a].resize(NUM_STATES);
		for (s = 0; s < NUM_STATES; s++)
		{
			alphafile.read(reinterpret_cast<char*>(&val), sizeof(double));
			alpha_vectors[a][s] = val;
		}
	}
}

pair<int, int> action2diff(int a)
{
	int x = 0;
	int y = 0;
	if (a == ACTION_N)
		y = 1;
	else if (a == ACTION_W)
		x = -1;
	else if (a == ACTION_S)
		y = -1;
	else if (a == ACTION_E)
		x = 1;
	else if (a == ACTION_NW)
	{
		y = 1;
		x = -1;
	}
	else if (a == ACTION_SW)
	{
		y = -1;
		x = -1;
	}
	else if (a == ACTION_NE)
	{
		y = 1;
		x = 1;
	}
	else if (a == ACTION_SE)
	{
		y = -1;
		x = 1;
	}
	return pair<int, int>(x, y);
}
