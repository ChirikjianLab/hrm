/** \author Sipu Ruan */

#pragma once

#include "FreeSpace.h"
#include "MultiBodyTree2D.h"
#include "hrm/geometry/SuperEllipse.h"

namespace hrm {

/** \class FreeSpace2D
 * \brief Compute free space in SE(2) */
class FreeSpace2D : public FreeSpaceComputator<MultiBodyTree2D, SuperEllipse> {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree2D object defining the robot
     * \param arena Geometric object of arena
     * \param obstacle Geometric object of obstacles */
    FreeSpace2D(const MultiBodyTree2D& robot,
                const std::vector<SuperEllipse>& arena,
                const std::vector<SuperEllipse>& obstacle);
    ~FreeSpace2D();

    void computeIntersectionInterval(
        const std::vector<std::vector<Coordinate>>& tLine) override;
};

}  // namespace hrm
