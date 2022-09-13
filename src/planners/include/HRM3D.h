#pragma once

#include "HighwayRoadMap.h"
#include "datastructure/include/FreeSpace3D.h"
#include "geometry/include/LineIntersection.h"
#include "geometry/include/MeshGenerator.h"
#include "geometry/include/TightFitEllipsoid.h"
#include "util/include/InterpolateSE3.h"

/** \class HRM3D
 * \brief Highway RoadMap planner for 3D rigid-body robot */
class HRM3D : public HighwayRoadMap<MultiBodyTree3D, SuperQuadrics> {
  public:
    /** \brief Constructor
     * \param robot MultibodyTree type defining the robot
     * \param arena vector of geometric types definint the planning arena
     * \param obs vector of geometric types defining obstacles
     * \param req PlanningRequest structure */
    HRM3D(const MultiBodyTree3D& robot, const std::vector<SuperQuadrics>& arena,
          const std::vector<SuperQuadrics>& obs, const PlanningRequest& req);

    virtual ~HRM3D();

    /** \brief Get the resulting solved path and the interpolated one */
    std::vector<std::vector<Coordinate>> getInterpolatedSolutionPath(
        const Index num);

    /** \brief Get free line segment at one specific C-layer
     * \param bd Pointer to Minkowski boundaries
     * \return Collision-free line segment as FreeSegment3D type */
    const FreeSegment3D& getFreeSegmentOneLayer(const BoundaryInfo* bd) {
        layerBound_ = *bd;
        freeSpacePtr_->computeCSpaceBoundaryMesh(&layerBound_);
        layerBoundMesh_ = freeSpacePtr_->getCSpaceBoundaryMesh();
        sweepLineProcess();
        return freeSegOneLayer_;
    }

    /** \brief Get Minkowski sums boundary mesh
     * \param idx Index of C-layer
     * \return Boundary mesh */
    const BoundaryMesh& getLayerBoundaryMesh(const Index idx) {
        return layerBoundMeshAll_.at(idx);
    }

    /** \brief Get Minkowski sums boundary
     * \param idx Index of C-layer
     * \return Boundary */
    const BoundaryInfo& getLayerBoundary(const Index idx) {
        return layerBoundAll_.at(idx);
    }

  protected:
    void constructOneLayer(const Index layerIdx) override;

    virtual void sampleOrientations() override;

    void sweepLineProcess() override;

    virtual void generateVertices(const Coordinate tx,
                                  const FreeSegment2D* freeSeg) override;

    /** \brief Connect within one C-layer
     * \param freeSeg 3D collision-free line segments */
    void connectOneLayer3D(const FreeSegment3D* freeSeg);

    virtual void connectMultiLayer() override;

    void connectExistLayer(const Index layerId) override;

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

    /** \param Pointer to class for constructing free space */
    std::shared_ptr<FreeSpace3D> freeSpacePtr_;
};
