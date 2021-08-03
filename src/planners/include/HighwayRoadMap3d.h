#ifndef HIGHWAYROADMAP3D_H
#define HIGHWAYROADMAP3D_H

#include "geometry/include/LineIntersection.h"
#include "geometry/include/TightFitEllipsoid.h"
#include "planners/include/HighwayRoadMap.h"
#include "util/include/InterpolateSE3.h"
#include "util/include/MultiBodyTree3D.h"

// FreeSegment3D collision-free line segments in 3D
struct FreeSegment3D {
    std::vector<double> tx;
    std::vector<FreeSegment2D> freeSegYZ;
};

class HighwayRoadMap3D : public HighwayRoadMap<MultiBodyTree3D, SuperQuadrics> {
  public:
    HighwayRoadMap3D(const MultiBodyTree3D& robot,
                     const std::vector<SuperQuadrics>& arena,
                     const std::vector<SuperQuadrics>& obs,
                     const PlanningRequest& req);

    virtual ~HighwayRoadMap3D();

  public:
    virtual void buildRoadmap() override;

    /** \brief sweep-line process to explore free space in one C-layer
     * \param Boundary Minkowski boundry of obstacles and arenas
     * \return collision-free segments info
     */
    FreeSegment3D sweepLine3D(const Boundary* bd);

    virtual void generateVertices(const double tx,
                                  const FreeSegment2D* freeSeg) override;

    /** \brief connect within one C-layer */
    void connectOneLayer3D(const FreeSegment3D* freeSeg);

    virtual void connectMultiLayer() override;

    /** \brief subroutine to first construct the middle C-layer */
    //    FreeSegment3D bridgeLayer(SuperQuadrics);
    std::vector<MeshMatrix> bridgeLayer(SuperQuadrics Ec);

    /** \brief get the resulting solved path and the interpolated one */
    std::vector<std::vector<double>> getInterpolatedSolutionPath(
        const unsigned int num);

  protected:
    void computeTFE(const Eigen::Quaterniond& v1, const Eigen::Quaterniond& v2,
                    std::vector<SuperQuadrics>* tfe);

    bool isSameLayerTransitionFree(const std::vector<double>& v1,
                                   const std::vector<double>& v2) override;

    virtual bool isMultiLayerTransitionFree(
        const std::vector<double>& v1, const std::vector<double>& v2) override;

    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) override;

    /** \brief check is one point is within C-free */
    bool isPtInCFree(const std::vector<MeshMatrix>* bdMesh,
                     const std::vector<double>& v);

    /** \brief uniform random sample SO(3) */
    void sampleSO3();

    virtual void setTransform(const std::vector<double>& v) override;

  public:
    /** \param q_r sampled orientations (Quaternion) of the robot */
    std::vector<Eigen::Quaterniond> q_r;

  protected:
    std::vector<MeshMatrix> layerBoundMesh_;

    std::vector<SuperQuadrics> tfe_;

    std::vector<FreeSegment3D> freeSeg_;
    std::vector<std::vector<MeshMatrix>> bridgeLayerBound_;
};

#endif  // HIGHWAYROADMAP3D_H
