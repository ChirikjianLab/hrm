#include <math.h>
#include <vector>

#define pi 3.1415926

// For almost uniform sampling //
// Sampe angles
std::vector<double> sampleSE(double a, double b, double ep, double D);

double updateTheta(double th, double a, double b, double ep, double D);
