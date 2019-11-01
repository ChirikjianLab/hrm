#ifndef TIGHTFITELLIPSOID_H
#define TIGHTFITELLIPSOID_H

#include "SuperEllipse.h"
#include "SuperQuadrics.h"

/*
 * \brief compute Minimum volume concentric ellipsoid
 */
SuperEllipse getMVCE2D(const std::vector<double>& a,
                       const std::vector<double>& b, const double thetaA,
                       const double thetaB, const int num);

SuperQuadrics getMVCE3D(const std::vector<double>& a,
                        const std::vector<double>& b,
                        const Eigen::Quaterniond& q_a,
                        const Eigen::Quaterniond& q_b, const int num);

/*
 * \brief compute tightly fitted ellipsoid for an ellipsoid with multiple
 * interpolated orientations
 */
SuperQuadrics getTFE3D(const std::vector<double>& a,
                       const Eigen::Quaterniond& q_a,
                       const Eigen::Quaterniond& q_b, const int N_step,
                       const int num);
#endif  // TIGHTFITELLIPSOID_H
