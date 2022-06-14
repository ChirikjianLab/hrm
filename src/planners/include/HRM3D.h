#pragma once

#include "HighwayRoadMap.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "geometry/include/LineIntersection.h"
#include "geometry/include/MeshGenerator.h"
#include "geometry/include/TightFitEllipsoid.h"
#include "util/include/InterpolateSE3.h"

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

/** \class HRM3D
 * \brief Highway RoadMap planner for 3D rigid-body robot */
class HRM3D : public HighwayRoadMap<MultiBodyTree3D, SuperQuadrics> {
  public:
    /** \brief Constructor
     * \param robot MultibodyTree type defining the robot
     * \param arena vector of geometric types definint the planning arena
     * \param obstacle vector of geometric types defining obstacles
     * \param req PlanningRequest structure */
    HRM3D(const MultiBodyTree3D& robot, const std::vector<SuperQuadrics>& arena,
          const std::vector<SuperQuadrics>& obs, const PlanningRequest& req);

    virtual ~HRM3D();

    /** \brief Get the resulting solved path and the interpolated one */
    std::vector<std::vector<Coordinate>> getInterpolatedSolutionPath(
        const Index num);

    /**
     * \brief Get free line segment at one specific C-layer
     * \param Boundary pointer to Minkowski boundaries
     * \return FreeSegment3D
     */
    FreeSegment3D getFreeSegmentOneLayer(const BoundaryInfo* bd) {
        layerBound_ = *bd;
        generateBoundaryMesh(&layerBound_, &layerBoundMesh_);
        sweepLineProcess();
        return freeSegOneLayer_;
    }

    /**
     * \brief Get Minkowski sums boundary mesh
     * \param idx Index of C-layer
     * \return Boundary mesh
     */
    BoundaryMesh getLayerBoundaryMesh(const Index idx) {
        return layerBoundMeshAll_.at(idx);
    }

    /**
     * \brief Get Minkowski sums boundary
     * \param idx Index of C-layer
     * \return Boundary
     */
    BoundaryInfo getLayerBoundary(const Index idx) {
        return layerBoundAll_.at(idx);
    }

    void constructOneLayer(const Index layerIdx) override;

    virtual void sampleOrientations() override;

    void sweepLineProcess() override;

    virtual void generateVertices(const Coordinate tx,
                                  const FreeSegment2D* freeSeg) override;

    /** \brief Connect within one C-layer
     * \param 3D collision-free line segments */
    void connectOneLayer3D(const FreeSegment3D* freeSeg);

    virtual void connectMultiLayer() override;

    void connectExistLayer(const Index layerId) override;

  protected:
    /** \brief Generate C-free boundary as mesh
     * \param bound C-arena/C-obstacle boundary points
     * \param boundMesh The generated mesh type */
    void generateBoundaryMesh(const BoundaryInfo* bound,
                              BoundaryMesh* boundMesh);

    void bridgeLayer() override;

    /** \brief Compute Tightly-Fitted Ellipsoid (TFE) to enclose robot parts
     * when rotating around its center
     * \param q1 Start orientation of the robot
     * \param q2 Goal orientation of the robot
     * \param tfe Resulting TFE that fully encloses the robot while under pure
     * rotational motions */
    virtual void computeTFE(const Eigen::Quaterniond& q1,
                            const Eigen::Quaterniond& q2,
                            std::vector<SuperQuadrics>* tfe);

    IntersectionInterval computeIntersections(
        const std::vector<Coordinate>& ty) override;

    bool isSameLayerTransitionFree(const std::vector<Coordinate>& v1,
                                   const std::vector<Coordinate>& v2) override;

    virtual bool isMultiLayerTransitionFree(
        const std::vector<Coordinate>& v1,
        const std::vector<Coordinate>& v2) override;

    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<Coordinate>& vertex, const Index k,
        const double radius) override;

    bool isPtInCFree(const Index bdIdx,
                     const std::vector<Coordinate>& v) override;

    /** \brief uniform random sample SO(3) */
    void sampleSO3();

    virtual void setTransform(const std::vector<Coordinate>& v) override;

  protected:
    /** \param Sampled orientations (Quaternion) of the robot */
    std::vector<Eigen::Quaterniond> q_;

    /** \param Boundary surface as mesh */
    BoundaryMesh layerBoundMesh_;

    /** \param Storage of boundary surface mesh */
    std::vector<BoundaryMesh> layerBoundMeshAll_;

    /** \param Collision-free line segments in one C-slice */
    FreeSegment3D freeSegOneLayer_;

    /** \param Minkowski boundaries mesh at bridge C-layer */
    std::vector<std::vector<MeshMatrix>> bridgeLayerBound_;
};
