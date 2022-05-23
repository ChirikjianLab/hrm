#pragma once

#include "HighwayRoadMap.h"
#include "datastructure/include/MultiBodyTree3D.h"
#include "geometry/include/LineIntersection.h"
#include "geometry/include/MeshGenerator.h"
#include "geometry/include/TightFitEllipsoid.h"
#include "util/include/InterpolateSE3.h"

/** \brief FreeSegment3D collision-free line segments in 3D */
struct FreeSegment3D {
    std::vector<Coordinate> tx;
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

    /** \brief get the resulting solved path and the interpolated one */
    std::vector<std::vector<Coordinate>> getInterpolatedSolutionPath(
        const Index num);

    /**
     * \brief get free line segment at one specific C-layer
     * \param Boundary pointer to Minkowski boundaries
     * \return FreeSegment3D
     */
    FreeSegment3D getFreeSegmentOneLayer(const Boundary* bd) {
        layerBound_ = *bd;
        generateBoundaryMesh(&layerBound_, &layerBoundMesh_);
        sweepLineProcess();
        return freeSegOneLayer_;
    }

    /**
     * \brief getLayerBoundaryMesh Get Minkowski sums boundary mesh
     * \param idx Index of C-layer
     * \return Boundary mesh
     */
    BoundaryMesh getLayerBoundaryMesh(const Index idx) {
        return layerBoundMeshAll_.at(idx);
    }

    /**
     * \brief getLayerBoundary Get Minkowski sums boundary
     * \param idx Index of C-layer
     * \return Boundary
     */
    Boundary getLayerBoundary(const Index idx) {
        return layerBoundAll_.at(idx);
    }

    void constructOneLayer(const Index layerIdx) override;

    virtual void sampleOrientations() override;

    void sweepLineProcess() override;

    virtual void generateVertices(const Coordinate tx,
                                  const FreeSegment2D* freeSeg) override;

    /** \brief connect within one C-layer */
    void connectOneLayer3D(const FreeSegment3D* freeSeg);

    virtual void connectMultiLayer() override;

    void connectExistLayer(const Index layerId) override;

  protected:
    void generateBoundaryMesh(const Boundary* bound, BoundaryMesh* boundMesh);

    void bridgeLayer() override;

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
    /** \param q_r sampled orientations (Quaternion) of the robot */
    std::vector<Eigen::Quaterniond> q_;

    /** \param layerBoundMesh_ boundary surface as mesh */
    BoundaryMesh layerBoundMesh_;
    std::vector<BoundaryMesh> layerBoundMeshAll_;

    /** \param freeSegOneLayer_ collision-free line segments in one C-slice */
    FreeSegment3D freeSegOneLayer_;

    /** \param Minkowski boundaries mesh at bridge C-layer */
    std::vector<std::vector<MeshMatrix>> bridgeLayerBound_;
};
