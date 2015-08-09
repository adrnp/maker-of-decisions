#include <iostream>
#include <stdio.h>
#include "pomdp.h"

// My write to file stuff...
#include <iostream>
#include <fstream>

using std::ofstream;

using std::cout;
using std::endl;


/**
 * Test that the stored alphas match what we have in the julia
 * Test has been passed.
 */
int test_stored_alphas()
{
	POMDP a;

	int i, j;
	for (i = 0; i < 41; i++)
	{
		for (j = 0; j < 41; j++)
		{
			if (a.stored_alphas[i][j] >= 10)
				cout << a.stored_alphas[i][j] << ",";
			else
				cout << " " << a.stored_alphas[i][j] << ",";
		}
		cout << endl;
	}
}


/**
 * Good. Matches julia
 * Sums to 1
 */
int test_obs_probs()
{
	POMDP a;
	int i;
	double sum = 0.0;
	for (i = 0; i < 4; i++)
	{
		cout << "obs_probs[" << i << "] = " << a.obs_probs[i] << endl;
		if (i > 0)
			sum += 2 * a.obs_probs[i];
		else
			sum += a.obs_probs[i];
	}
	cout << "sum = " << sum << endl;
}


/**
 * Tests the observation function.
 * Cases are compared with the julia implementation
 * (See ~/research/GridPOMDP/test_O.jl)
 *
 * Has passed all the tests that I've given, it, but I am weary
 */

int test_O()
{
	initialize_pomdp();

	double prob;
	vector<int> sp (4);
	sp[0] = 4;
	sp[1] = 4;
	sp[2] = 8;
	sp[3] = 5;

	int o;
	for (o = 0; o < 36; o++)
	{
		prob = O(sp, o);
		cout << "o = " << o << ", prob is " << prob << endl; 
	}
}


/**
 * Assume for now that transition works
 */
int test_T()
{
}


int test_ind2state()
{
	vector<int> state(4);
	state[0] = 0;
	state[1] = 0;
	state[2] = 0;
	state[3] = 0;

	ind2state(state, 42);

	cout << "state is:\n";
	cout << "[" << state[0] << ", " << state[1] << ", " << state[2] << ", " << state[3] << "]\n";
}


/**
 * Tests that state2ind and ind2state match.
 * They do match. Assume both are correct.
 */
int test_state2ind()
{
	vector<int> state(4);
	state[0] = 0;
	state[1] = 0;
	state[2] = 0;
	state[3] = 0;

	int test_ind = 197;
	ind2state(state, test_ind);
	int ind = state2ind(state);

	cout << "test_ind, ind = " << test_ind << ", " << ind << endl;
}

int test_bupdate()
{

}

/**
 * Draws 2d belief matrices
 */
void printmat(vector<vector<double> >& mat)
{
	int x, y;
	int numrows = mat.size();
	int numcols = mat[0].size();

	for (y = numcols; y >=0 ; y--)
	{
		for (x = 0; x < numrows-1; x++)
		{
			printf("%.3f,",mat[x][y]);
		}
		printf("%.4f\n",mat[numrows-1][y]);
	}
}

int test_naiive()
{
	int rssi = 32.0; // test that shit
	double bearing;
	vector<vector<double> > b2mat;
	pair<float, float> naxy;

	bearing = 212.0;
	naxy = get_next_pomdp_action(bearing, rssi);
	printf("North = %.1f, East = %.1f\n", naxy.first,naxy.second);
	b2mat = getb2mat();
	printmat(b2mat);

	bearing = 257.0;
	naxy = get_next_pomdp_action(bearing, rssi);
	printf("North = %.1f, East = %.1f\n", naxy.first,naxy.second);
	b2mat = getb2mat();
	printmat(b2mat);
}


int testy(double &bearing)
{
	cout << "bearing = " << bearing << endl;
}

int test_out_file()
{
	ofstream myfile;
	myfile.open("rar.txt");
}


int main()
{
	//test_pomdp();
	//test_stored_alphas();
	//test_obs_probs();
	//test_O();
	//test_ind2state();
	//test_state2ind();
	//cout << obs_probs[2] << endl;

	test_naiive();
	

	/*
	int rar;
	rar = reverse_obs(45);
	cout << "rar = " << rar << endl;
	rar = reverse_obs(135);
	cout << "rar = " << rar << endl;
	rar = reverse_obs(225);
	cout << "rar = " << rar << endl;
	rar = reverse_obs(315);
	cout << "rar = " << rar << endl;
	*/
	//initialize_belief();
	
	//test_O();
}
