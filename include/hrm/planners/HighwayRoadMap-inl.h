/** \authors Sipu Ruan, Qianli Ma */

#pragma once

#include "HighwayRoadMap.h"
#include "hrm/datastructure/Interval.h"
#include "hrm/util/DistanceMetric.h"

#include <algorithm>
#include <chrono>
#include <list>
#include <random>

namespace hrm {
namespace planners {

using Clock = std::chrono::high_resolution_clock;
using Durationd = std::chrono::duration<double>;

/** \brief visitor that terminates when we find the goal */
struct AStarFoundGoal {};

/** \class AStarGoalVisitor
 * \brief Class for goal visitor in A* search */
template <class Vertex>
class AStarGoalVisitor : public boost::default_astar_visitor {
  public:
    /** \brief Constructor
     * \param goal Goal vertex */
    AStarGoalVisitor(Vertex goal) : goal_(goal) {}

    /** \brief Examine whether reaches goal
     * \param u Current vertex
     * \param g Graph structure */
    template <class Graph>
    void examine_vertex(Vertex u, Graph& g) {
        if (u == goal_) {
            throw AStarFoundGoal();
        }
    }

  private:
    /** \brief Goal vertex */
    Vertex goal_;
};

/** \class HighwayRoadMap
 * \brief Superclass for HRM-based planners */
template <class RobotType, class ObjectType>
HighwayRoadMap<RobotType, ObjectType>::HighwayRoadMap(
    const RobotType& robot, const std::vector<ObjectType>& arena,
    const std::vector<ObjectType>& obs, const PlanningRequest& req)
    : robot_(robot),
      arena_(arena),
      obs_(obs),
      start_(req.start),
      goal_(req.goal),
      param_(req.parameters),
      isRobotRigid_(req.isRobotRigid) {}

template <class RobotType, class ObjectType>
HighwayRoadMap<RobotType, ObjectType>::~HighwayRoadMap() {}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::plan(const double timeLim) {
    // Plan and timing
    auto start = Clock::now();
    buildRoadmap();
    res_.planningTime.buildTime += Durationd(Clock::now() - start).count();

    start = Clock::now();
    search();
    res_.planningTime.searchTime += Durationd(Clock::now() - start).count();

    res_.planningTime.totalTime =
        res_.planningTime.buildTime + res_.planningTime.searchTime;

    // Refine existing roadmap
    while (!res_.solved && res_.planningTime.totalTime < timeLim) {
        refineExistRoadmap(timeLim);
    }

    // Get solution path
    if (res_.solved) {
        res_.solutionPath.solvedPath = getSolutionPath();
        res_.solutionPath.interpolatedPath =
            getInterpolatedSolutionPath(param_.numPoint);
    }
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::buildRoadmap() {
    sampleOrientations();

    // Construct roadmap
    for (size_t i = 0; i < param_.numSlice; ++i) {
        // construct one C-slice
        constructOneSlice(i);

        // Record vertex index at each C-slice
        numVertex_.slice = res_.graphStructure.vertex.size();
        vertexIdx_.push_back(numVertex_);
    }

    // Connect adjacent slices using bridge C-slice
    connectMultiSlice();
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::search() {
    // Construct the roadmap
    const Index num_vtx = res_.graphStructure.vertex.size();
    AdjGraph g(num_vtx);

    for (Index i = 0; i < res_.graphStructure.edge.size(); ++i) {
        boost::add_edge(Index(res_.graphStructure.edge[i].first),
                        Index(res_.graphStructure.edge[i].second),
                        Weight(res_.graphStructure.weight[i]), g);
    }

    // Locate the nearest vertex for start and goal in the roadmap
    const std::vector<Vertex> idx_s = getNearestNeighborsOnGraph(
        start_, param_.numSearchNeighbor, param_.searchRadius);
    const std::vector<Vertex> idx_g = getNearestNeighborsOnGraph(
        goal_, param_.numSearchNeighbor, param_.searchRadius);

    // Search for shortest path in the searching regions
    Index num;
    for (Vertex idxS : idx_s) {
        for (Vertex idxG : idx_g) {
            std::vector<Vertex> p(num_vertices(g));
            std::vector<double> d(num_vertices(g));

            try {
                boost::astar_search(
                    g, idxS,
                    [this, idxG](Vertex v) {
                        return vectorEuclidean(
                            res_.graphStructure.vertex[v],
                            res_.graphStructure.vertex[idxG]);
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
                res_.solutionPath.cost = 0.0;
                res_.solutionPath.PathId.push_back(int(idxG));
                while (res_.solutionPath.PathId[num] != int(idxS) &&
                       num <= num_vtx) {
                    res_.solutionPath.PathId.push_back(
                        int(p[size_t(res_.solutionPath.PathId[num])]));
                    res_.solutionPath.cost +=
                        res_.graphStructure
                            .weight[size_t(res_.solutionPath.PathId[num])];
                    num++;
                }
                std::reverse(std::begin(res_.solutionPath.PathId),
                             std::end(res_.solutionPath.PathId));

                if (num == num_vtx + 1) {
                    res_.solutionPath.PathId.clear();
                    res_.solutionPath.cost = INFINITY;
                } else {
                    res_.solved = true;
                    return;
                }
            }
        }
    }
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::refineExistRoadmap(
    const double timeLim) {
    isRefine_ = true;

    vertexIdxAll_.push_back(vertexIdx_);
    vertexIdx_.clear();

    param_.numLineX *= 2;
    param_.numLineY *= 2;

    for (size_t i = 0; i < param_.numSlice; ++i) {
        auto start = Clock::now();

        // construct refined C-slice
        constructOneSlice(i);
        numVertex_.slice = res_.graphStructure.vertex.size();
        vertexIdx_.push_back(numVertex_);

        // Connect with existing slices
        connectExistSlice(i);

        res_.planningTime.buildTime += Durationd(Clock::now() - start).count();

        // Search
        start = Clock::now();
        search();
        res_.planningTime.searchTime += Durationd(Clock::now() - start).count();

        res_.planningTime.totalTime =
            res_.planningTime.buildTime + res_.planningTime.searchTime;

        if (res_.solved || res_.planningTime.totalTime > timeLim) {
            return;
        }
    }

    isRefine_ = false;
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::connectOneSlice2D(
    const FreeSegment2D& freeSeg) {
    // Add connections to edge list
    Index n1 = 0;
    Index n2 = 0;

    for (size_t i = 0; i < freeSeg.ty.size(); ++i) {
        n1 = numVertex_.plane.at(i);

        for (size_t j1 = 0; j1 < freeSeg.xM[i].size(); ++j1) {
            // Connect vertex within the same sweep line
            if (j1 != freeSeg.xM[i].size() - 1) {
                if (std::fabs(freeSeg.xU[i][j1] - freeSeg.xL[i][j1 + 1]) <
                    1e-6) {
                    res_.graphStructure.edge.push_back(
                        std::make_pair(n1 + j1, n1 + j1 + 1));
                    res_.graphStructure.weight.push_back(vectorEuclidean(
                        res_.graphStructure.vertex[n1 + j1],
                        res_.graphStructure.vertex[n1 + j1 + 1]));
                }
            }

            // Connect vertex btw adjacent sweep lines
            if (i != freeSeg.ty.size() - 1) {
                n2 = numVertex_.plane.at(i + 1);

                for (Index j2 = 0; j2 < freeSeg.xM[i + 1].size(); ++j2) {
                    if (isSameSliceTransitionFree(
                            res_.graphStructure.vertex[n1 + j1],
                            res_.graphStructure.vertex[n2 + j2])) {
                        // Direct success connection
                        res_.graphStructure.edge.push_back(
                            std::make_pair(n1 + j1, n2 + j2));
                        res_.graphStructure.weight.push_back(vectorEuclidean(
                            res_.graphStructure.vertex[n1 + j1],
                            res_.graphStructure.vertex[n2 + j2]));
                    } else {
                        bridgeVertex(n1 + j1, n2 + j2);
                    }
                }
            }
        }
    }
}

template <class RobotType, class ObjectType>
std::vector<std::vector<Coordinate>>
HighwayRoadMap<RobotType, ObjectType>::getSolutionPath() {
    std::vector<std::vector<Coordinate>> path;
    auto poseSize = res_.graphStructure.vertex.at(0).size();

    // Start pose
    start_.resize(poseSize);
    path.push_back(start_);

    // Iteratively store intermediate poses along the solved path
    for (auto pathId : res_.solutionPath.PathId) {
        path.push_back(res_.graphStructure.vertex.at(size_t(pathId)));
    }

    // Goal pose
    goal_.resize(poseSize);
    path.push_back(goal_);

    return path;
}

template <class RobotType, class ObjectType>
std::vector<std::vector<double>>
HighwayRoadMap<RobotType, ObjectType>::getInterpolatedSolutionPath(
    const unsigned int num) {
    std::vector<std::vector<Coordinate>> interpPath =
        res_.solutionPath.solvedPath;
    return interpPath;
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::bridgeVertex(const Index idx1,
                                                         const Index idx2) {
    const auto v1 = res_.graphStructure.vertex.at(idx1);
    const auto v2 = res_.graphStructure.vertex.at(idx2);

    // Generate new bridge vertex
    auto vNew1 = v1;
    vNew1.at(2) = v2.at(2);
    auto vNew2 = v2;
    vNew2.at(2) = v1.at(2);

    auto vNew = v1;

    // Check validity of potential connections
    if (isSameSliceTransitionFree(v1, vNew1) &&
        isSameSliceTransitionFree(vNew1, v2)) {
        vNew = vNew1;
    } else if (isSameSliceTransitionFree(v1, vNew2) &&
               isSameSliceTransitionFree(vNew2, v2)) {
        vNew = vNew2;
    } else {
        return;
    }

    // Add new bridge vertex to graph is new connection is valid
    int idxNew = res_.graphStructure.vertex.size();
    res_.graphStructure.vertex.push_back(vNew);
    res_.graphStructure.edge.push_back(std::make_pair(idx1, idxNew));
    res_.graphStructure.weight.push_back(vectorEuclidean(v1, vNew));
    res_.graphStructure.edge.push_back(std::make_pair(idxNew, idx2));
    res_.graphStructure.weight.push_back(vectorEuclidean(vNew, v2));
}

}  // namespace planners
}  // namespace hrm
