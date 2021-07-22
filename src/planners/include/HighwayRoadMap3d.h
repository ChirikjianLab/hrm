#ifndef HIGHWAYROADMAP3D_H
#define HIGHWAYROADMAP3D_H

#include "src/geometry/include/SuperQuadrics.h"
#include "src/geometry/include/TightFitEllipsoid.h"
#include "src/util/include/Interval.h"
#include "util/include/DistanceMetric.h"
#include "util/include/InterpolateSE3.h"
#include "util/include/LineIntersection.h"

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/Time.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>

#include <algorithm>
#include <limits>
#include <vector>

using Weight = boost::property<boost::edge_weight_t, double>;
using AdjGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                          boost::no_property, Weight>;
using Vertex = AdjGraph::vertex_descriptor;
using edge_descriptor = AdjGraph::edge_descriptor;
using vertex_iterator = AdjGraph::vertex_iterator;
using Edge = std::vector<std::pair<int, int>>;
using WeightMap = boost::property_map<AdjGraph, boost::edge_weight_t>::type;

// cf_cell: collision-free points
struct cf_cellYZ {
    std::vector<double> ty;
    std::vector<std::vector<double>> zL;
    std::vector<std::vector<double>> zU;
    std::vector<std::vector<double>> zM;
};

struct cf_cell3D {
    std::vector<double> tx;
    std::vector<cf_cellYZ> cellYZ;
};

// boundary: Minkowski boundary points for obstacles and arenas
struct boundary3D {
    std::vector<Eigen::MatrixXd> bd_s;
    std::vector<Eigen::MatrixXd> bd_o;
};

struct option3D {
    size_t N_layers, N_dx, N_dy, N_o, N_s;
    std::vector<double> Lim;
};

class HighwayRoadMap3D {
  public:
    HighwayRoadMap3D(SuperQuadrics robot,
                     std::vector<std::vector<double>> endpt,
                     std::vector<SuperQuadrics> arena,
                     std::vector<SuperQuadrics> obs, option3D opt);
    virtual ~HighwayRoadMap3D();

  public:
    /**
     * \brief main function for planning
     */
    virtual void plan();

    /**
     * \brief main function for building the roadmap
     */
    virtual void buildRoadmap();

    /**
     * \brief compute Minkowski sum boundary
     */
    virtual boundary3D boundaryGen();

    /**
     * \brief sweep-line process for cell decomposition, vertical lines parallel
     * to z-axis
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
    virtual void generateVertices(const double tx, const cf_cellYZ* cellYZ);

    cf_cellYZ cfLine(std::vector<double> ty, Eigen::MatrixXd z_s_L,
                     Eigen::MatrixXd z_s_U, Eigen::MatrixXd z_o_L,
                     Eigen::MatrixXd z_o_U);

    /*
     * \brief connect within one C-layer
     */
    void connectOneLayer(cf_cell3D cell);

    /*
     * \brief subroutine to first connect vertices within on
     * sweep-plane(vertical to x-axis)
     */
    void connectOnePlane(const cf_cellYZ* cellYZ);

    /**
     * \brief connect within adjacent C-layers, using the idea of "bridge
     * C-layer"
     */
    virtual void connectMultiLayer();

    /**
     * \brief subroutine to first construct the middle C-layer
     */
    //    cf_cell3D midLayer(SuperQuadrics);
    std::vector<MeshMatrix> midLayer(SuperQuadrics);

    /**
     * \brief check whether connection between V1 and V2 within one C-layer is
     * valid through line segment V1-V2 and C-obstacle mesh intersection
     * checking
     */
    bool isOneLayerTransitionFree(const std::vector<double>& V1,
                                  const std::vector<double>& V2);

    /**
     * \brief check whether connection between V1 and V2 is valid through
     * interpolation
     */
    virtual bool isTransitionFree(const std::vector<double>& V1,
                                  const std::vector<double>& V2);

    /**
     * \brief check is one point is within C-free
     */
    bool isPtInCFree(const std::vector<MeshMatrix>* bdMesh,
                     const std::vector<double>& V);

    /**
     * \brief graph search using a-star algorithm
     */
    void search();

    /**
     * \brief get the resulting solved path and the interpolated one
     */
    std::vector<std::vector<double>> getSolutionPath();
    std::vector<std::vector<double>> getInterpolatedSolutionPath(
        const unsigned int num);

    /**
     * \brief uniform random sample SO(3)
     */
    void sampleSO3();

    /**
     * \brief transformation for robot
     */
    virtual void setTransform(const std::vector<double>& V);

  private:
    /**
     * \brief enhanced cell decomposition, connect vertices within one
     * collision-free line segment, all connections between vertexes are within
     * one convex cell
     */
    cf_cellYZ enhanceDecomp(cf_cellYZ cell);

    /**
     * \brief find the nearest neighbors of a pose on the graph
     * \param v the queried vertex
     * \param k number of neighbors
     * \param r radius of a neighboring ball around v
     */
    std::vector<Vertex> getNearestNeighborsOnGraph(const std::vector<double>& v,
                                                   const size_t k,
                                                   const double r);

    /**
     * \brief query whether a point is within a collision-free line segment
     */
    // bool isPtinCFLine(std::vector<double>, std::vector<double>);

    /**
     * \brief Variables
     * N_o       : number of obstacles;
     * N_s       : number of arenas;
     * N_dx      : number of sweep planes within one C-layer;
     * N_dy      : number of sweep lines within one sweep plane;
     * Lim       : planning bound limit;
     * N_layers  : number of C-layers;
     * q_r       : sampled orientations (Quaternion) of the robot;
     * N_v_layer : number of vertex in each layer;
     * N_step    : number of interpolated orientations between two vertices at
                different C-layers, default = 3

     * graph     : a structure consisting of vertex and edges;
     * Cost      : cost of the searched path;
     * Endpt     : start and goal configurations;
     * Path      : valid path of motions;
     * planTime  : planning time: roadmap building time and path search time
     */
  public:
    SuperQuadrics Robot;
    std::vector<SuperQuadrics> Arena;
    std::vector<SuperQuadrics> Obs;
    std::vector<std::vector<double>> Endpt;

    // Parameters for the roadmap
    size_t N_o;
    size_t N_s;
    size_t N_dx;
    size_t N_dy;
    size_t N_layers;
    std::vector<double> Lim;
    std::vector<Eigen::Quaterniond> q_r;
    unsigned int N_step = 10;

    struct vertexIdx {
        size_t layer;
        std::vector<size_t> plane;
        std::vector<std::vector<size_t>> line;
    } N_v;

    // graph: vector of vertices, vector of connectable edges
    struct graph {
        std::vector<std::vector<double>> vertex;
        Edge edge;
        std::vector<double> weight;
    } vtxEdge;

    // Vertex index info
    std::vector<vertexIdx> vtxId;

    // Solution info
    struct SolutionPathInfo {
        std::vector<int> PathId;
        std::vector<std::vector<double>> solvedPath;
        std::vector<std::vector<double>> interpolatedPath;
        double Cost = std::numeric_limits<double>::infinity();
    } solutionPathInfo;

    struct Time {
      public:
        double buildTime, searchTime, totalTime;
    } planTime;

    bool flag = false;

  protected:
    std::vector<MeshMatrix> CLayerBound;
    std::vector<SuperQuadrics> mid;
    //    cf_cell3D mid_cell;
    std::vector<MeshMatrix> midLayerBound;
    std::vector<double> midVtx;
};

#endif  // HIGHWAYROADMAP3D_H
