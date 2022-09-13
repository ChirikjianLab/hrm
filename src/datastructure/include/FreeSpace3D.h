#pragma once

#include "FreeSpace.h"
#include "MultiBodyTree3D.h"
#include "geometry/include/SuperQuadrics.h"

/** \brief Collision-free line segments in 3D */
struct FreeSegment3D {
    /** \brief x-coordinates of the 3D line segments */
    std::vector<Coordinate> tx;

    /** \brief Vector of line segments in yz-plane as FreeSegment2D type */
    std::vector<FreeSegment2D> freeSegYZ;
};

/** \brief Mesh representation of Minkowski operations */
struct BoundaryMesh {
    /** \brief Mesh of arena */
    std::vector<MeshMatrix> arena;

    /** \brief Mesh of obstacles */
    std::vector<MeshMatrix> obstacle;
};

/** \class FreeSpace3D
 * \brief Compute free space in SE(3) */
class FreeSpace3D : public FreeSpaceComputator<MultiBodyTree3D, SuperQuadrics> {
  public:
    /** \brief Constructor
     * \param robot MultiBodyTree3D object defining the robot
     * \param arena Geometric object of arena
     * \param obstacle Geometric object of obstacles */
    FreeSpace3D(const MultiBodyTree3D& robot,
                const std::vector<SuperQuadrics>& arena,
                const std::vector<SuperQuadrics>& obstacle);
    ~FreeSpace3D() {}

    /** \brief Get C-free boundary as mesh
     * \return The generated mesh as BoundMesh type */
    const BoundaryMesh& getCSpaceBoundaryMesh() { return cSpaceBoundaryMesh_; }

    void computeIntersectionInterval(
        const std::vector<std::vector<Coordinate>>& tLine) override;

    /** \brief Compute C-free boundary as mesh
     * \param bound C-arena/C-obstacle boundary points*/
    void computeCSpaceBoundaryMesh(const BoundaryInfo* bound);

  private:
    BoundaryMesh cSpaceBoundaryMesh_;
};
