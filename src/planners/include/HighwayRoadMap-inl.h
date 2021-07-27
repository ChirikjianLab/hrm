#ifndef HIGHWAYROADMAP_INL_H
#define HIGHWAYROADMAP_INL_H

#include "HighwayRoadMap.h"
#include "util/include/DistanceMetric.h"

#include "ompl/util/Time.h"

#include <algorithm>
#include <list>
#include <random>

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
HighwayRoadMap<RobotType, ObjectType>::HighwayRoadMap(
    const RobotType& robot, const std::vector<ObjectType>& arena,
    const std::vector<ObjectType>& obs, const PlanningRequest& req)
    : robot_(robot),
      arena_(arena),
      obs_(obs),
      start_(req.start),
      goal_(req.goal),
      param_(req.planner_parameters),
      N_o(obs.size()),
      N_s(arena.size()) {}

template <class RobotType, class ObjectType>
HighwayRoadMap<RobotType, ObjectType>::~HighwayRoadMap() {}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::plan() {
    // Plan and timing
    ompl::time::point start = ompl::time::now();
    buildRoadmap();
    res_.planning_time.buildTime =
        ompl::time::seconds(ompl::time::now() - start);

    start = ompl::time::now();
    search();
    res_.planning_time.searchTime =
        ompl::time::seconds(ompl::time::now() - start);

    res_.planning_time.totalTime =
        res_.planning_time.buildTime + res_.planning_time.searchTime;

    // Get solution path
    res_.solution_path.solvedPath = getSolutionPath();
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::search() {
    std::vector<Vertex> idx_s;
    std::vector<Vertex> idx_g;
    size_t num;

    // Construct the roadmap
    size_t num_vtx = res_.graph_structure.vertex.size();
    AdjGraph g(num_vtx);

    for (size_t i = 0; i < res_.graph_structure.edge.size(); ++i) {
        boost::add_edge(size_t(res_.graph_structure.edge[i].first),
                        size_t(res_.graph_structure.edge[i].second),
                        Weight(res_.graph_structure.weight[i]), g);
    }

    // Locate the nearest vertex for start and goal in the roadmap
    idx_s = getNearestNeighborsOnGraph(start_, param_.NUM_SEARCH_NEIGHBOR,
                                       param_.SEARCH_RADIUS);
    idx_g = getNearestNeighborsOnGraph(goal_, param_.NUM_SEARCH_NEIGHBOR,
                                       param_.SEARCH_RADIUS);

    // Search for shortest path in the searching regions
    for (Vertex idxS : idx_s) {
        for (Vertex idxG : idx_g) {
            std::vector<Vertex> p(num_vertices(g));
            std::vector<double> d(num_vertices(g));

            try {
                boost::astar_search(
                    g, idxS,
                    [this, idxG](Vertex v) {
                        return vectorEuclidean(
                            res_.graph_structure.vertex[v],
                            res_.graph_structure.vertex[idxG]);
                    },
                    boost::predecessor_map(
                        boost::make_iterator_property_map(
                            p.begin(), get(boost::vertex_index, g)))
                        .distance_map(make_iterator_property_map(
                            d.begin(), get(boost::vertex_index, g)))
                        .visitor(AStarGoalVisitor<Vertex>(idxG)));
            } catch (AStarFoundGoal found) {
                // Record path and cost
                num = 0;
                res_.solution_path.cost = 0.0;
                res_.solution_path.PathId.push_back(int(idxG));
                while (res_.solution_path.PathId[num] != int(idxS) &&
                       num <= num_vtx) {
                    res_.solution_path.PathId.push_back(
                        int(p[size_t(res_.solution_path.PathId[num])]));
                    res_.solution_path.cost +=
                        res_.graph_structure
                            .weight[size_t(res_.solution_path.PathId[num])];
                    num++;
                }
                std::reverse(std::begin(res_.solution_path.PathId),
                             std::end(res_.solution_path.PathId));

                if (num == num_vtx + 1) {
                    res_.solution_path.PathId.clear();
                    res_.solution_path.cost = inf;
                } else {
                    res_.solved = true;
                    return;
                }
            }
        }
    }
}

template <class RobotType, class ObjectType>
std::vector<std::vector<double>>
HighwayRoadMap<RobotType, ObjectType>::getSolutionPath() {
    std::vector<std::vector<double>> path;
    auto poseSize = res_.graph_structure.vertex.at(0).size();

    // Start pose
    start_.resize(poseSize);
    path.push_back(start_);

    // Iteratively store intermediate poses along the solved path
    for (auto pathId : res_.solution_path.PathId) {
        path.push_back(res_.graph_structure.vertex.at(size_t(pathId)));
    }

    // Goal pose
    goal_.resize(poseSize);
    path.push_back(goal_);

    return path;
}

template <class RobotType, class ObjectType>
cf_cell2D HighwayRoadMap<RobotType, ObjectType>::enhanceDecomp(cf_cell2D cell) {
    // Make sure all connections between vertexes are within one convex cell
    cf_cell2D cell_new = cell;

    for (size_t i = 0; i < cell.ty.size() - 1; ++i) {
        for (size_t j1 = 0; j1 < cell.xM[i].size(); ++j1) {
            for (size_t j2 = 0; j2 < cell.xM[i + 1].size(); ++j2) {
                if (cell_new.xM[i][j1] < cell_new.xL[i + 1][j2] &&
                    cell_new.xU[i][j1] >= cell_new.xL[i + 1][j2]) {
                    cell_new.xU[i].push_back(cell_new.xL[i + 1][j2]);
                    cell_new.xL[i].push_back(cell_new.xL[i + 1][j2]);
                    cell_new.xM[i].push_back(cell_new.xL[i + 1][j2]);
                } else if (cell_new.xM[i][j1] > cell_new.xU[i + 1][j2] &&
                           cell_new.xL[i][j1] <= cell_new.xU[i + 1][j2]) {
                    cell_new.xU[i].push_back(cell_new.xU[i + 1][j2]);
                    cell_new.xL[i].push_back(cell_new.xU[i + 1][j2]);
                    cell_new.xM[i].push_back(cell_new.xU[i + 1][j2]);
                }

                if (cell_new.xM[i + 1][j2] < cell_new.xL[i][j1] &&
                    cell_new.xU[i + 1][j2] >= cell_new.xL[i][j1]) {
                    cell_new.xU[i + 1].push_back(cell_new.xL[i][j1]);
                    cell_new.xL[i + 1].push_back(cell_new.xL[i][j1]);
                    cell_new.xM[i + 1].push_back(cell_new.xL[i][j1]);
                } else if (cell_new.xM[i + 1][j2] > cell_new.xU[i][j1] &&
                           cell_new.xL[i + 1][j2] <= cell_new.xU[i][j1]) {
                    cell_new.xU[i + 1].push_back(cell_new.xU[i][j1]);
                    cell_new.xL[i + 1].push_back(cell_new.xU[i][j1]);
                    cell_new.xM[i + 1].push_back(cell_new.xU[i][j1]);
                }
            }
        }

        sort(cell_new.xL[i].begin(), cell_new.xL[i].end(),
             [](double a, double b) { return a < b; });
        sort(cell_new.xU[i].begin(), cell_new.xU[i].end(),
             [](double a, double b) { return a < b; });
        sort(cell_new.xM[i].begin(), cell_new.xM[i].end(),
             [](double a, double b) { return a < b; });
    }

    return cell_new;
}

#endif  // HIGHWAYROADMAP_INL_H
