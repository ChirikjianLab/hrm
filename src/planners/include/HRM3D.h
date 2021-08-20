#pragma once

#include "datastructure/include/MultiBodyTree3D.h"
#include "geometry/include/LineIntersection.h"
#include "geometry/include/TightFitEllipsoid.h"
#include "planners/include/HighwayRoadMap.h"
#include "util/include/InterpolateSE3.h"

/** \brief FreeSegment3D collision-free line segments in 3D */
struct FreeSegment3D {
    std::vector<double> tx;
    std::vector<FreeSegment2D> freeSegYZ;
};

/** \brief BoundaryMesh mesh representation of Minkowski operations */
struct BoundaryMesh {
    std::vector<MeshMatrix> arena;
    std::vector<MeshMatrix> obstacle;
};

class HRM3D : public HighwayRoadMap<MultiBodyTree3D, SuperQuadrics> {
  public:
    HRM3D(const MultiBodyTree3D& robot, const std::vector<SuperQuadrics>& arena,
          const std::vector<SuperQuadrics>& obs, const PlanningRequest& req);

    virtual ~HRM3D();

  public:
    /** \brief get the resulting solved path and the interpolated one */
    std::vector<std::vector<double>> getInterpolatedSolutionPath(
        const unsigned int num);

    /**
     * \brief get free line segment at one specific C-layer
     * \param Boundary pointer to Minkowski boundaries
     * \return FreeSegment3D
     */
    FreeSegment3D getFreeSegmentOneLayer(const Boundary* bd) {
        layerBound_ = *bd;
        sweepLineProcess();
        return freeSegOneLayer_;
    }

    virtual void buildRoadmap() override;

    virtual void sampleOrientations() override;

    void sweepLineProcess() override;

    virtual void generateVertices(const double tx,
                                  const FreeSegment2D* freeSeg) override;

    /** \brief connect within one C-layer */
    void connectOneLayer3D(const FreeSegment3D* freeSeg);

    virtual void connectMultiLayer() override;

    void connectExistLayer() override;

  protected:
    void generateBoundaryMesh(const Boundary* bound, BoundaryMesh* boundMesh);

    void bridgeLayer() override;

    void computeTFE(const Eigen::Quaterniond& v1, const Eigen::Quaterniond& v2,
                    std::vector<SuperQuadrics>* tfe);

    IntersectionInterval computeIntersections(
        const std::vector<double>& ty) override;

    bool isSameLayerTransitionFree(const std::vector<double>& v1,
                                   const std::vector<double>& v2) override;

    virtual bool isMultiLayerTransitionFree(
        const std::vector<double>& v1, const std::vector<double>& v2) override;

    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) override;

    bool isPtInCFree(const int bdIdx, const std::vector<double>& v) override;

    /** \brief uniform random sample SO(3) */
    void sampleSO3();

    virtual void setTransform(const std::vector<double>& v) override;

  public:
    /** \param q_ storage of sampled orientations (Quaternion) of the robot
     */
    std::vector<Eigen::Quaterniond> q_;

    /** \param quatNew_ sampled new orientations (Quaternion) of the robot */
    std::vector<Eigen::Quaterniond> qNew_;

  protected:
    BoundaryMesh layerBoundMesh_;
    std::vector<BoundaryMesh> layerBoundMeshAll_;

    FreeSegment3D freeSegOneLayer_;

    /** \param Minkowski boundaries mesh at bridge C-layer */
    std::vector<std::vector<MeshMatrix>> bridgeLayerBound_;
};
