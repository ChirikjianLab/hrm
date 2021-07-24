#ifndef HIGHWAYROADMAP3D_H
#define HIGHWAYROADMAP3D_H

#include "src/geometry/include/TightFitEllipsoid.h"
#include "src/planners/include/HighwayRoadMap.h"
#include "src/util/include/Interval.h"
#include "util/include/InterpolateSE3.h"
#include "util/include/LineIntersection.h"

#include <ompl/datastructures/NearestNeighbors.h>

// cf_cell: collision-free points
struct cf_cell3D {
    std::vector<double> tx;
    std::vector<cf_cell2D> cellYZ;
};

class HighwayRoadMap3D : public HighwayRoadMap<SuperQuadrics, SuperQuadrics> {
  public:
    HighwayRoadMap3D(const SuperQuadrics& robot,
                     const std::vector<SuperQuadrics>& arena,
                     const std::vector<SuperQuadrics>& obs,
                     const PlanningRequest& req);

    virtual ~HighwayRoadMap3D();

  public:
    /** \brief main function for building the roadmap */
    virtual void buildRoadmap() override;

    /** \brief compute Minkowski sum boundary */
    virtual boundary boundaryGen() override;

    /**
     * \brief sweep-line process for cell decomposition, vertical lines
     * parallel to z-axis
     * \param bd_s, bd_o Minkowski boundry of obstacles and arenas
     * \return collision-free cells info
     */
    cf_cell3D sweepLineZ(std::vector<Eigen::MatrixXd> bd_s,
                         std::vector<Eigen::MatrixXd> bd_o);

    /**
     * \brief subroutine for generating collision-free vertices on the yz-plane
     * \param ty a vector of incremented y-coordinates
     * \param z_s_L, z_s_U, z_o_L, z_o_U upper(U) and lower(L) z-coordinates of
     * the intersection for arena(s) and obstacles(o), size = num of
     * y-coordinates X num of objects
     * \return collision-free cells info
     */
    virtual void generateVertices(const double tx, const cf_cell2D* cellYZ);

    cf_cell2D cfLine(std::vector<double> ty, Eigen::MatrixXd z_s_L,
                     Eigen::MatrixXd z_s_U, Eigen::MatrixXd z_o_L,
                     Eigen::MatrixXd z_o_U);

    /** \brief connect within one C-layer */
    void connectOneLayer(cf_cell3D cell);
    void connectOneLayer(cf_cell2D cell) override;

    /** \brief subroutine to first connect vertices within on
     * sweep-plane(vertical to x-axis) */
    void connectOnePlane(const cf_cell2D* cellYZ);

    /** \brief connect within adjacent C-layers, using the idea of "bridge
     * C-layer" */
    virtual void connectMultiLayer() override;

    /** \brief subroutine to first construct the middle C-layer */
    //    cf_cell3D midLayer(SuperQuadrics);
    std::vector<MeshMatrix> midLayer(SuperQuadrics);

    /** \brief check whether connection between V1 and V2 within one C-layer is
     * valid through line segment V1-V2 and C-obstacle mesh intersection
     * checking */
    bool isOneLayerTransitionFree(const std::vector<double>& V1,
                                  const std::vector<double>& V2);

    /** \brief check whether connection between V1 and V2 is valid through
     * interpolation */
    virtual bool isTransitionFree(const std::vector<double>& V1,
                                  const std::vector<double>& V2);

    /** \brief check is one point is within C-free */
    bool isPtInCFree(const std::vector<MeshMatrix>* bdMesh,
                     const std::vector<double>& V);

    /** \brief get the resulting solved path and the interpolated one */
    std::vector<std::vector<double>> getInterpolatedSolutionPath(
        const unsigned int num);

    /** \brief uniform random sample SO(3) */
    void sampleSO3();

    /** \brief transformation for robot */
    virtual void setTransform(const std::vector<double>& V);

  private:
    /** \brief find the nearest neighbors of a pose on the graph */
    std::vector<Vertex> getNearestNeighborsOnGraph(
        const std::vector<double>& vertex, const size_t k,
        const double radius) override;

    /**
     * \brief query whether a point is within a collision-free line segment
     */
    // bool isPtinCFLine(std::vector<double>, std::vector<double>);

  public:
    /** \param q_r sampled orientations (Quaternion) of the robot */
    std::vector<Eigen::Quaterniond> q_r;

    struct vertexIdx {
        size_t layer;
        std::vector<size_t> plane;
        std::vector<std::vector<size_t>> line;
    } N_v;

    // Vertex index info
    std::vector<vertexIdx> vtxId;

  protected:
    std::vector<MeshMatrix> CLayerBound;
    std::vector<SuperQuadrics> mid;
    std::vector<MeshMatrix> midLayerBound;
    std::vector<double> midVtx;
};

#endif  // HIGHWAYROADMAP3D_H
