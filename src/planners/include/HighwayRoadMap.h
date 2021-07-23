#ifndef HIGHWAYROADMAP_H
#define HIGHWAYROADMAP_H

#include "planners/include/PlanningRequest.h"
#include "util/include/DistanceMetric.h"

#include <ompl/util/Time.h>

#include "Eigen/Dense"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>

#include <algorithm>
#include <limits>
#include <list>
#include <random>

using Weight = boost::property<boost::edge_weight_t, double>;
using AdjGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                          boost::no_property, Weight>;
using Vertex = AdjGraph::vertex_descriptor;
using edge_descriptor = AdjGraph::edge_descriptor;
using vertex_iterator = AdjGraph::vertex_iterator;
using WeightMap = boost::property_map<AdjGraph, boost::edge_weight_t>::type;

using Edge = std::vector<std::pair<int, int>>;

static const double pi = 3.1415926;

/** \brief freeSegment collision-free line segments */
struct cf_cell2D {
  public:
    std::vector<double> ty;
    std::vector<std::vector<double>> xL;
    std::vector<std::vector<double>> xU;
    std::vector<std::vector<double>> xM;
};

/** \brief boundary Minkowski boundary points for obstacles and arenas */
struct boundary {
  public:
    std::vector<Eigen::MatrixXd> bd_s;
    std::vector<Eigen::MatrixXd> bd_o;
};

/** \brief visitor that terminates when we find the goal */
struct AStarFoundGoal {};

template <class Vertex>
class AStarGoalVisitor : public boost::default_astar_visitor {
  public:
    AStarGoalVisitor(Vertex goal) : goal_(goal) {}
    template <class Graph>
    void examine_vertex(Vertex u, Graph& g) {
        if (u == goal_) {
            throw AStarFoundGoal();
        }
    }

  private:
    Vertex goal_;
};

/** \class HighwayRoadMap superclass for HRM-based planners */
template <class RobotType, class ObjectType>
class HighwayRoadMap {
  public:
    HighwayRoadMap(const RobotType& robot, const std::vector<ObjectType>& arena,
                   const std::vector<ObjectType>& obs,
                   const PlanningRequest& req)
        : robot_(robot),
          arena_(arena),
          obs_(obs),
          start_(req.start),
          goal_(req.goal),
          param_(req.planner_parameters),
          N_o(obs.size()),
          N_s(arena.size()) {}

    virtual ~HighwayRoadMap() {}

  public:
    virtual void plan() {
        ompl::time::point start = ompl::time::now();
        buildRoadmap();
        planTime.buildTime = ompl::time::seconds(ompl::time::now() - start);

        start = ompl::time::now();
        search();
        planTime.searchTime = ompl::time::seconds(ompl::time::now() - start);

        planTime.totalTime = planTime.buildTime + planTime.searchTime;
    }

    virtual void buildRoadmap() = 0;
    virtual boundary boundaryGen() = 0;
    virtual void connectOneLayer(cf_cell2D cell) = 0;
    virtual void connectMultiLayer() = 0;

    void search() {
        Vertex idx_s, idx_g, num;

        // Construct the roadmap
        size_t num_vtx = vtxEdge.vertex.size();
        AdjGraph g(num_vtx);

        for (size_t i = 0; i < vtxEdge.edge.size(); ++i) {
            boost::add_edge(size_t(vtxEdge.edge[i].first),
                            size_t(vtxEdge.edge[i].second),
                            Weight(vtxEdge.weight[i]), g);
        }

        // Locate the nearest vertex for start and goal in the roadmap
        idx_s = getNearestVtxOnGraph(start_);
        idx_g = getNearestVtxOnGraph(goal_);

        // Search for shortest path
        std::vector<Vertex> p(num_vertices(g));
        std::vector<double> d(num_vertices(g));
        planTime.totalTime = planTime.buildTime + planTime.searchTime;
        try {
            boost::astar_search(
                g, idx_s,
                [this, idx_g](Vertex v) {
                    return vectorEuclidean(vtxEdge.vertex[v],
                                           vtxEdge.vertex[idx_g]);
                },
                boost::predecessor_map(
                    boost::make_iterator_property_map(
                        p.begin(), get(boost::vertex_index, g)))
                    .distance_map(make_iterator_property_map(
                        d.begin(), get(boost::vertex_index, g)))
                    .visitor(AStarGoalVisitor<Vertex>(idx_g)));
        } catch (AStarFoundGoal found) {
            // Record path and cost
            num = 0;
            Paths.push_back(int(idx_g));
            while (Paths[num] != int(idx_s) && num <= num_vtx) {
                Paths.push_back(int(p[size_t(Paths[num])]));
                Cost += d[size_t(Paths[num])];
                num++;
            }

            std::reverse(std::begin(Paths), std::end(Paths));

            if (num == num_vtx + 1) {
                Paths.clear();
                Cost = std::numeric_limits<double>::infinity();
            }
        }
    }

  protected:
    virtual cf_cell2D enhanceDecomp(cf_cell2D cell) = 0;
    virtual size_t getNearestVtxOnGraph(std::vector<double> v) = 0;

  public:
    /** \param graph vector of vertices, vector of connectable edges */
    struct graph {
      public:
        std::vector<std::vector<double>> vertex;
        Edge edge;
        std::vector<double> weight;
    } vtxEdge;

    /** \param Cost cost of the searched path */
    double Cost = 0.0;

    /** \param Path valid path of motions */
    std::vector<int> Paths;

    /** \param Time roadmap building time and path search time */
    struct Time {
      public:
        double buildTime = 0.0;
        double searchTime = 0.0;
        double totalTime = 0.0;
    } planTime;

    RobotType robot_;
    std::vector<ObjectType> arena_;
    std::vector<ObjectType> obs_;

    std::vector<double> start_;
    std::vector<double> goal_;

    /** \param N_o number of obstacles */
    size_t N_o;

    /** \param N_s number of arenas */
    size_t N_s;

    /** \param N_v_layer number of vertex in each layer */
    std::vector<size_t> N_v_layer;

    PlannerParameter param_;
};

#endif  // HIGHWAYROADMAP_H
