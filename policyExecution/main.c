#include <stdio.h>

#include "policyExecution.h"

#define TESTING 1

int main(void)
{
	int bestAction;

	double alphaVectors[VECTOR_LENGTH][NUM_VECTORS];
	double alphaActions[NUM_VECTORS];
	readPolicy("test.xml", alphaVectors, alphaActions);

	/* Starting location of the vehicle */
	int xStart = 2;
	int yStart = 2;

	/* Initialize initial state belief */
	double belief[] = {[0 ... VECTOR_LENGTH-1] = 0.0};
	initBelief(belief, xStart, yStart);
	double bprime[] = {[0 ... VECTOR_LENGTH-1] = 1.0/VECTOR_LENGTH};

	/* Get first action */
	bestAction = getAction(belief, alphaVectors);
	printf("best action = %i\n", bestAction);


	/* Tell vehicle to take bestAction.
	 * Wait for its response.
	 * It will tell us where it now thinks it is
	 */
	/* TODO: Handle interface with vehicle */


	/* Now vehicle has responded with some idea of where it is */
	// 1. Update your belief
	// 2. Use this belief to determine the next action
	updateBelief(belief, bprime, bestAction, 2);
	

	return 0;
}
