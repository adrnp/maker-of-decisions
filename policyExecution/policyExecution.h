// Must know number of alpha vectors and the length of each before starting
#define NUM_VECTORS 6
#define GRID_SIZE 5
#define VECTOR_LENGTH 625
#define NUM_STATE_VARS 4

#define ALTITUDE 150
#define UNIT_ANGLE 5

#define SIGMA 1
#define NUM_SIGMAS 3
#define CELL_SIZE 20

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
