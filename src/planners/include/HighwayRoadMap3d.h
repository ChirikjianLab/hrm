#ifndef HIGHWAYROADMAP3D_H
#define HIGHWAYROADMAP3D_H

#include "src/geometry/include/TightFitEllipsoid.h"
#include "src/planners/include/HighwayRoadMap.h"
#include "src/util/include/Interval.h"
#include "util/include/InterpolateSE3.h"
#include "util/include/LineIntersection.h"

#include <ompl/datastructures/NearestNeighbors.h>

// FreeSegment3D collision-free line segments in 3D
struct FreeSegment3D {
    std::vector<double> tx;
    std::vector<FreeSegment2D> cellYZ;
};

/** \brief vertexIdx vertex index at each C-layer, sweep line */
struct vertexIdx {
    size_t layer;
    std::vector<size_t> plane;
    std::vector<std::vector<size_t>> line;
};

class HighwayRoadMap3D : public HighwayRoadMap<SuperQuadrics, SuperQuadrics> {
  public:
    HighwayRoadMap3D(const SuperQuadrics& robot,
                     const std::vector<SuperQuadrics>& arena,
                     const std::vector<SuperQuadrics>& obs,
                     const PlanningRequest& req);

    virtual ~HighwayRoadMap3D();

  public:
    virtual void buildRoadmap() override;
    virtual Boundary boundaryGen() override;

    /** \brief sweep-line process for cell decomposition, vertical lines
     * parallel to z-axis
     * \param bd_s, bd_o Minkowski boundry of obstacles and arenas
     * \return collision-free cells info
     */
    FreeSegment3D sweepLineZ(const Boundary* bd);

    /** \brief subroutine for generating collision-free vertices on the yz-plane
     * \param ty a vector of incremented y-coordinates
     * \param z_s_L, z_s_U, z_o_L, z_o_U upper(U) and lower(L) z-coordinates of
     * the intersection for arena(s) and obstacles(o), size = num of
     * y-coordinates X num of objects
     * \return collision-free cells info
     */
    virtual void generateVertices(const double tx, const FreeSegment2D* cellYZ);

    FreeSegment2D cfLine(std::vector<double> ty, Eigen::MatrixXd z_s_L,
                         Eigen::MatrixXd z_s_U, Eigen::MatrixXd z_o_L,
                         Eigen::MatrixXd z_o_U);

    /** \brief connect within one C-layer */
    void connectOneLayer3D(const FreeSegment3D* cell);

    /** \brief subroutine to first connect vertices within on sweep-plane
     * (vertical to x-axis) */
    void connectOneLayer2D(const FreeSegment2D* cellYZ) override;

    /** \brief connect within adjacent C-layers, using the idea of "bridge
     * C-layer" */
    virtual void connectMultiLayer() override;

    /** \brief subroutine to first construct the middle C-layer */
    //    FreeSegment3D midLayer(SuperQuadrics);
    std::vector<MeshMatrix> midLayer(SuperQuadrics);

    /** \brief uniform random sample SO(3) */
    void sampleSO3();

    /** \brief transformation for robot */
    virtual void setTransform(const std::vector<double>& V);

    /** \brief get the resulting solved path and the interpolated one */
    std::vector<std::vector<double>> getInterpolatedSolutionPath(
        const unsigned int num);

  protected:
    bool isSameLayerTransitionFree(const std::vector<double>& V1,
                                   const std::vector<double>& V2) override;

    virtual bool isMultiLayerTransitionFree(
        const std::vector<double>& V1, const std::vector<double>& V2) override;

    /** \brief check is one point is within C-free */
    bool isPtInCFree(const std::vector<MeshMatrix>* bdMesh,
                     const std::vector<double>& V);

    /** \brief find the nearest neighbors of a pose on the graph */
    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) override;

  public:
    /** \param q_r sampled orientations (Quaternion) of the robot */
    std::vector<Eigen::Quaterniond> q_r;

    // Vertex index info
    std::vector<vertexIdx> vtxId;
    vertexIdx N_v;

  protected:
    std::vector<MeshMatrix> CLayerBound;
    std::vector<SuperQuadrics> mid;
    std::vector<MeshMatrix> midLayerBound;
    std::vector<double> midVtx;
};

#endif  // HIGHWAYROADMAP3D_H
