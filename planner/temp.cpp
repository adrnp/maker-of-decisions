
/**
 * Determines the state that, coupled with action a, led to sp_state
 * Assumes deterministic transitions
 * Assumes s_state has the last two elements same as sp_state
 */
int anti_action(vector<int>& s_state, vector<int>& sp_state, int a)
{
	int xadd = 0;
	int yadd = 0;

	/* East-West */
	if (a == 1 || a == 4 || a == 5)
	{
		/* westerly action, but in reverse */
		xadd = 1;
	}
	else if (a == 3 || a == 6 || a == 7)
	{
		/* easterly action, but in reverse */
		xadd = -1;
	}


	/* North-South */
	if (a == 0 || a == 4 || a == 6)
	{
		/* northerly action, but in reverse */
		yadd = -1;
	}
	else if (a == 2 || a == 5 || a == 7)
	{
		/* southerly action, but in reverse */
		yadd = 1;
	}

	/* Make the changes */
	s_state[0] += xadd;
	s_state[1] += yadd;

	/* Check that this be valid */
	if (s_state[0] > 20 || s_state[1] > 20)
	{
		/* invalid previous state */
	}
	else if (s_state[0] < 0 || s_state[1] < 0)
}
