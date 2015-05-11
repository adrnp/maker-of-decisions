#include <iostream>

#include "pomdp.h"

using std::cout;
using std::endl;


int test_pomdp()
{
	POMDP a;
	cout << "grid_size = " << a.grid_size << endl;
	cout << "num_actions = " << a.num_actions << endl;
}


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
	POMDP m;

	int a, o;
	vector<int> sp(4);
	double prob;
	sp[2] = 10;
	sp[3] = 10;

	/* First trial: Do we get null obs over the jammer, even if we spin? */
	/* Trial successfully passed */
	a = ACTION_ROTATE;
	o = NULL_OBS;
	sp[0] = 10;
	sp[1] = 10;
	prob = O(a, sp, o, m.stored_alphas, m.obs_probs);
	cout << "a = " << a;
	cout << "; sp = [" << sp[0] << "," << sp[1] << "," << sp[2] << "," << sp[3] << "]";
	cout << "; o = " << o << " => ";
	cout << "prob = " << prob << endl;

	/* Second trial: */
	cout << "\nTEST 2" << endl;
	a = ACTION_ROTATE;
	sp[0] = 0;
	sp[1] = 0;
	for (o = 0; o < 9; o++) 
	{
		prob = O(a, sp, o, m.stored_alphas, m.obs_probs);
		cout << "a = " << a << "; ";
		cout << "sp = [" << sp[0] << "," << sp[1] << "," << sp[2] << "," << sp[3] << "]; ";
		cout << "o = " << o << " => ";
		cout << "prob = " << prob << endl;
	}

	/* Third test: */
	/* What happens at the boundary? */
	/* Works, but I'm not convinced */
	cout << "\nTEST 3" << endl;
	a = ACTION_ROTATE;
	sp[0] = 0;
	sp[1] = 10;
	for (o = 32; o < 36; o++) 
	{
		prob = O(a, sp, o, m.stored_alphas, m.obs_probs);
		cout << "a = " << a << "; ";
		cout << "sp = [" << sp[0] << "," << sp[1] << "," << sp[2] << "," << sp[3] << "]; ";
		cout << "o = " << o << " => ";
		cout << "prob = " << prob << endl;
	}
	for (o = 0; o < 5; o++) 
	{
		prob = O(a, sp, o, m.stored_alphas, m.obs_probs);
		cout << "a = " << a << "; ";
		cout << "sp = [" << sp[0] << "," << sp[1] << "," << sp[2] << "," << sp[3] << "]; ";
		cout << "o = " << o << " => ";
		cout << "prob = " << prob << endl;
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


int main()
{
	//test_pomdp();
	//test_stored_alphas();
	//test_obs_probs();
	test_O();
	//test_ind2state();
	//test_state2ind();

}
