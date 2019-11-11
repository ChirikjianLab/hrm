#ifndef TIGHTFITELLIPSOID_H
#define TIGHTFITELLIPSOID_H

#include "SuperEllipse.h"
#include "SuperQuadrics.h"

/*
 * \brief compute Minimum volume concentric ellipsoid
 */
SuperEllipse getMVCE2D(const std::vector<double>& a,
                       const std::vector<double>& b, const double thetaA,
                       const double thetaB, const unsigned int num);

SuperQuadrics getMVCE3D(const std::vector<double>& a,
                        const std::vector<double>& b,
                        const Eigen::Quaterniond& quatA,
                        const Eigen::Quaterniond& quatB,
                        const unsigned int num);

/*
 * \brief compute tightly fitted ellipsoid for an ellipsoid with multiple
 * interpolated orientations
 */
SuperEllipse getTFE2D(const std::vector<double>& a, const double thetaA,
                      const double thetaB, const unsigned int numStep,
                      const unsigned int num);

SuperQuadrics getTFE3D(const std::vector<double>& a,
                       const Eigen::Quaterniond& quatA,
                       const Eigen::Quaterniond& quatB,
                       const unsigned int numStep, const unsigned int num);
#endif  // TIGHTFITELLIPSOID_H
