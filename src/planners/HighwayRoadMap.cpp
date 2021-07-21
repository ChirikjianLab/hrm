#include "include/HighwayRoadMap.h"
#include "util/include/Interval.h"
#include "util/include/LineIntersection.h"
#include "util/include/PointInPoly.h"

#include <fstream>
#include <iostream>
#include <list>
#include <random>

HighwayRoadMap::HighwayRoadMap(const SuperEllipse& robot,
                               const std::vector<std::vector<double>>& endpt,
                               const std::vector<SuperEllipse>& arena,
                               const std::vector<SuperEllipse>& obs,
                               const param& param)
    : Robot(robot),
      Arena(arena),
      Obs(obs),
      Endpt(endpt),
      Cost(0.0),
      polyVtx(param.polyVtx),
      N_o(long(param.N_o)),
      N_s(long(param.N_s)),
      N_dy(long(param.N_dy)),
      N_layers(param.N_layers),
      infla(param.infla),
      numMidSample(param.sampleNum),
      Lim(param.Lim) {}

HighwayRoadMap::~HighwayRoadMap() {}

void HighwayRoadMap::plan() {
    ompl::time::point start = ompl::time::now();
    buildRoadmap();
    planTime.buildTime = ompl::time::seconds(ompl::time::now() - start);

    start = ompl::time::now();
    search();
    planTime.searchTime = ompl::time::seconds(ompl::time::now() - start);
}

// exception for termination
struct AStarFoundGoal {};

// visitor that terminates when we find the goal
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

void HighwayRoadMap::search() {
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
    idx_s = getNearestVtxOnGraph(Endpt[0]);
    idx_g = getNearestVtxOnGraph(Endpt[1]);

    // Search for shortest path
    std::vector<Vertex> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));
    boost::astar_search(
        g, idx_s,
        [this, idx_g](Vertex v) {
            return vectorEuclidean(vtxEdge.vertex[v], vtxEdge.vertex[idx_g]);
        },
        boost::predecessor_map(boost::make_iterator_property_map(
                                   p.begin(), get(boost::vertex_index, g)))
            .distance_map(make_iterator_property_map(
                d.begin(), get(boost::vertex_index, g)))
            .visitor(AStarGoalVisitor<Vertex>(idx_g)));

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
