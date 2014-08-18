
/* Dynamics */
int getNeighbors(int neighbors[][NUM_STATE_VARS], int s[]);
double transitionFunction(int s_prime[], int s[], int a);

/* Observations */
double obsFunction(int o, int s_prime[], int a);


/* Beliefs */
int initBelief(double *belief, int xStart, int yStart);
int updateBelief(double *b, double *bprime, int a, int o);

/* Policy */
int readPolicy(char *xmlFileString, double alphaVectors[][NUM_VECTORS], double alphaActions[]);
double getAction(double vector[], double array[][NUM_VECTORS]);

/* Math */
int ind2state(int state[], int dims[], int ind);
int state2ind(int stateSizes[], int s[]);
double dot_product(double v[], double u[], int n);
