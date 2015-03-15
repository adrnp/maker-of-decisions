#include <vector>
#include <cstdlib>
#include <unistd.h>
#include <cmath>

using std::vector;
using std::pair;
using std::string;

/* pass in two vectors to this function */
double get_bearing(vector<double> angles, vector<double> gains);

/* helper functions */
pair<vector<double>, vector<double> > get_gains(string filename);
double mean(vector<double> vec);
double std_dev(vector<double> vec, double mean);
vector<double> normalize(vector<double> gains);
double cc_helper(vector<double> & angles, vector<double> & gains, vector<double> & ref_angles, vector<double> & ref_gains, double shift);
double cross_correlate(vector<double> & angles, vector<double> & gains, vector<double> & ref_angles, vector<double> & ref_gains);
double interp_gain(double angle, double shift, vector<double> & ref_angles, vector<double> & ref_gains);