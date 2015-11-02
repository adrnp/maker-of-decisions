#include "pomdp.h"

#include <fstream>

// I don't think I really use this
POMDP::POMDP()
{
	grid_size = 21;
	num_states = 21^4;
	num_actions = 8;
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

int make_bin_probs(vector<vector<vector<double> > >& bin_probs)
{
	/* Resize bin_probs */
	int i;
	int j;
	int k;
	int mat_size = 2*GRID_SIZE - 1;
	bin_probs.resize(mat_size);
	for (i = 0; i < mat_size; i++)
	{
		bin_probs[i].resize(mat_size);
		for (j = 0; j < mat_size; j++)
		{
			bin_probs[i][j].resize(37);
		}
	}

	/* Now read in the file we want: */
	double val;
	std::ifstream myfile;
	myfile.open("bin_probs_21.bin");

	for (i = 0; i < mat_size; i++)
	{
		for (j = 0; j < mat_size; j++)
		{
			for (k = 0; k < 37; k++)
			{
				myfile.read(reinterpret_cast<char*>(&val), sizeof(double));
				bin_probs[i][j][k] = val;
			}
		}
	}

	return 0;
}
