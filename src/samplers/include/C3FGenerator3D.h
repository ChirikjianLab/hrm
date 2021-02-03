#ifndef C3FGENERATOR3D_H
#define C3FGENERATOR3D_H

#include "geometry/include/FreeSpace3D.h"
#include "util/include/MultiBodyTree3D.h"

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/geometric/SimpleSetup.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class C3FGenerator3D {
  public:
    C3FGenerator3D(MultiBodyTree3D *robot, std::vector<SuperQuadrics> *arena,
                   std::vector<SuperQuadrics> *obstacle, parameters3D *param,
                   og::SimpleSetupPtr ss);
    virtual ~C3FGenerator3D();

  public:
    std::vector<const ob::State *> getValidStates() { return validStateSet_; }

    double getBuildTime() { return buildTime_; }

    /*
     * \brief Build a set of precomputed valid states
     * compute C-obstacle boundaries via closed-form Minkowski sums,
     * valid states comes from multiple random samples on collision-free
     * sweep line segments
     */
    virtual void fromSweepLine();

    /*
     * \brief Build a set of precomputed valid states
     * compute C-obstacle boundaries via closed-form Minkowski sums,
     * valid states comes from C-obstacle boundaries
     */
    virtual void fromBoundary();

  protected:
    // Parameters
    MultiBodyTree3D *robot_;
    std::vector<SuperQuadrics> *arena_;
    std::vector<SuperQuadrics> *obstacle_;
    parameters3D *param_;
    og::SimpleSetupPtr ss_;

    // Pre-computed free sample seeds set
    std::vector<const ob::State *> validStateSet_;
    double buildTime_ = 0.0;
};

#endif  // C3FGENERATOR3D_H
