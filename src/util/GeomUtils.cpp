#include "include/GeomUtils.h"

// For almost uniform sampling //
// Sampe angles
std::vector<double> sampleSE(double a, double b, double ep, double D) {
  std::vector<double> theta;
  std::vector<double> ang;
  theta.push_back(0.0);
  size_t n = 0;
  size_t max_iter = 1e5;

  for (size_t i = 0; i < max_iter; i++) {
    if (theta[n] >= pi / 2) {
      break;
    }

    double th = updateTheta(theta[n], a, b, ep, D);
    n++;
    theta.push_back(th);
  }
  n++;
  theta.push_back(pi / 2);

  ang = theta;
  for (int k = 1; k <= 3; k++) {
    for (size_t i = 0; i < theta.size(); i++) {
      ang.push_back(theta[i] + k * pi / 2);
    }
  }

  return ang;
}

double updateTheta(double th, double a, double b, double ep, double D) {
  double th_ep = 0.01, dth;
  if (th <= th_ep) {
    dth = pow(D / b + std::pow(th, ep), 1 / ep) - th;
  } else {
    if (pi / 2 - th <= th_ep) {
      dth = pow(D / a + pow(pi / 2 - th, ep), 1 / ep) - (pi / 2 - th);
    } else {
      dth = (D / ep * cos(th) + sin(th)) /
            sqrt(pow(a, 2) * expFun(th, 2 * ep, 0) * expFun(th, 4, 1) +
                 pow(b, 2) * expFun(th, 2 * ep, 1) * expFun(th, 4, 0));
    }
  }

  return th + dth;
}
