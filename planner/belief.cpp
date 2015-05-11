/**
 * belief.cpp
 *
 * Handles the belief updating
 */

#include "pomdp.h"

/**
 * Updates bprime
 */
int update_belief(vector<double>& bp, vector<double>& b, int a, int o, vector<vector<int> >& stored_alphas, vector<double>& obsProbs)
{
	int num_states = 194481;

	double inner_sum, oprob;

	vector<int> s(4);
	vector<int> sp(4);
	int s_index;
	int sp_index;

	/* loop over all states */
	for (sp_index = 0; sp_index < num_states, sp_index++)
	{
		ind2state(sp, sp_index);
		inner_sum = 0.0;

		oprob = O(a, sp, o, stored_alphas, obs_probs);

		/* loop over all states that could have reached here */
		s[2] = sp[2];
		s[3] = sp[3];
		for (x = 0; x < 21; x++)
		{
			s[0] = x;
			for (y = 0; y < 21; y++)
			{
				s[1] = y;
				s_index = state2ind(s);
				inner_sum += T(s, a, sp) * b[s_index];
			}
		}
		bp[sp_index] = inner_sum * oprob;
	}

	/* TODO: we must now normalize
	 *  This means divide by the sum of this
	 *  Also, detect failure (sum is zero)
	 *  If we get a failure... return -1
	 */
}
