#ifndef GX_RANDOM
#define GX_RANDOM

#include <random>

#ifndef RANDOM_SEED
#define RANDOM_SEED 10
#endif

double random_uniform();
void boxMuller(double stdev, double &x, double &y);

#endif