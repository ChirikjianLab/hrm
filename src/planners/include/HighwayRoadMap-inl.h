#pragma once

#include "HighwayRoadMap.h"
#include "datastructure/include/Interval.h"
#include "util/include/DistanceMetric.h"

#include <algorithm>
#include <chrono>
#include <list>
#include <random>

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
      param_(req.planner_parameters),
      N_o(obs.size()),
      N_s(arena.size()),
      isRobotRigid_(req.is_robot_rigid) {}

template <class RobotType, class ObjectType>
HighwayRoadMap<RobotType, ObjectType>::~HighwayRoadMap() {}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::plan(const double timeLim) {
    // Plan and timing
    auto start = Clock::now();
    buildRoadmap();
    res_.planning_time.buildTime += Durationd(Clock::now() - start).count();

    start = Clock::now();
    search();
    res_.planning_time.searchTime += Durationd(Clock::now() - start).count();

    res_.planning_time.totalTime =
        res_.planning_time.buildTime + res_.planning_time.searchTime;

    // Refine existing roadmap
    while (!res_.solved && res_.planning_time.totalTime < timeLim) {
        refineExistRoadmap(timeLim);
    }

    // Get solution path
    if (res_.solved) {
        res_.solution_path.solvedPath = getSolutionPath();
        res_.solution_path.interpolatedPath =
            getInterpolatedSolutionPath(param_.NUM_POINT);
    }
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::buildRoadmap() {
    sampleOrientations();

    // Construct roadmap
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // construct one C-layer
        constructOneLayer(i);

        // Record vertex index at each C-layer
        N_v.layer = res_.graph_structure.vertex.size();
        vtxId_.push_back(N_v);
    }

    // Connect adjacent layers using bridge C-layer
    connectMultiLayer();
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::search() {
    // Construct the roadmap
    const Index num_vtx = res_.graph_structure.vertex.size();
    AdjGraph g(num_vtx);

    for (Index i = 0; i < res_.graph_structure.edge.size(); ++i) {
        boost::add_edge(Index(res_.graph_structure.edge[i].first),
                        Index(res_.graph_structure.edge[i].second),
                        Weight(res_.graph_structure.weight[i]), g);
    }

    // Locate the nearest vertex for start and goal in the roadmap
    const std::vector<Vertex> idx_s = getNearestNeighborsOnGraph(
        start_, param_.NUM_SEARCH_NEIGHBOR, param_.SEARCH_RADIUS);
    const std::vector<Vertex> idx_g = getNearestNeighborsOnGraph(
        goal_, param_.NUM_SEARCH_NEIGHBOR, param_.SEARCH_RADIUS);

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
void HighwayRoadMap<RobotType, ObjectType>::refineExistRoadmap(
    const double timeLim) {
    isRefine_ = true;

    vtxIdAll_.push_back(vtxId_);
    vtxId_.clear();

    param_.NUM_LINE_X *= 2;
    param_.NUM_LINE_Y *= 2;

    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        auto start = Clock::now();

        // construct refined C-layer
        constructOneLayer(i);
        N_v.layer = res_.graph_structure.vertex.size();
        vtxId_.push_back(N_v);

        // Connect with existing layers
        connectExistLayer(i);

        res_.planning_time.buildTime += Durationd(Clock::now() - start).count();

        // Search
        start = Clock::now();
        search();
        res_.planning_time.searchTime +=
            Durationd(Clock::now() - start).count();

        res_.planning_time.totalTime =
            res_.planning_time.buildTime + res_.planning_time.searchTime;

        if (res_.solved || res_.planning_time.totalTime > timeLim) {
            return;
        }
    }

    isRefine_ = false;
}

template <class RobotType, class ObjectType>
BoundaryInfo HighwayRoadMap<RobotType, ObjectType>::boundaryGen() {
    BoundaryInfo bd;

    // Minkowski boundary points
    std::vector<BoundaryPoints> bdAux;
    for (auto arena : arena_) {
        bdAux = robot_.minkSum(&arena, -1);
        for (auto bdEach : bdAux) {
            bd.arena.push_back(bdEach);
        }
        bdAux.clear();
    }
    for (auto obs : obs_) {
        bdAux = robot_.minkSum(&obs, 1);
        for (auto bdEach : bdAux) {
            bd.obstacle.push_back(bdEach);
        }
        bdAux.clear();
    }

    return bd;
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::connectOneLayer2D(
    const FreeSegment2D* freeSeg) {
    // Add connections to edge list
    Index n1 = 0;
    Index n2 = 0;

    for (size_t i = 0; i < freeSeg->ty.size(); ++i) {
        n1 = N_v.plane.at(i);

        for (size_t j1 = 0; j1 < freeSeg->xM[i].size(); ++j1) {
            // Connect vertex within the same sweep line
            if (j1 != freeSeg->xM[i].size() - 1) {
                if (std::fabs(freeSeg->xU[i][j1] - freeSeg->xL[i][j1 + 1]) <
                    1e-6) {
                    res_.graph_structure.edge.push_back(
                        std::make_pair(n1 + j1, n1 + j1 + 1));
                    res_.graph_structure.weight.push_back(vectorEuclidean(
                        res_.graph_structure.vertex[n1 + j1],
                        res_.graph_structure.vertex[n1 + j1 + 1]));
                }
            }

            // Connect vertex btw adjacent sweep lines
            if (i != freeSeg->ty.size() - 1) {
                n2 = N_v.plane.at(i + 1);

                for (Index j2 = 0; j2 < freeSeg->xM[i + 1].size(); ++j2) {
                    if (isSameLayerTransitionFree(
                            res_.graph_structure.vertex[n1 + j1],
                            res_.graph_structure.vertex[n2 + j2])) {
                        // Direct success connection
                        res_.graph_structure.edge.push_back(
                            std::make_pair(n1 + j1, n2 + j2));
                        res_.graph_structure.weight.push_back(vectorEuclidean(
                            res_.graph_structure.vertex[n1 + j1],
                            res_.graph_structure.vertex[n2 + j2]));
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
std::vector<std::vector<double>>
HighwayRoadMap<RobotType, ObjectType>::getInterpolatedSolutionPath(
    const unsigned int num) {
    std::vector<std::vector<Coordinate>> interpPath =
        res_.solution_path.solvedPath;
    return interpPath;
}

template <class RobotType, class ObjectType>
FreeSegment2D HighwayRoadMap<RobotType, ObjectType>::computeFreeSegment(
    const std::vector<Coordinate>& ty, const IntersectionInterval* intersect) {
    FreeSegment2D freeLineSegment;
    Interval op;
    std::vector<Interval> interval[ty.size()];

    // y-coord
    freeLineSegment.ty = ty;

    // CF line segment for each ty
    for (Index i = 0; i < ty.size(); ++i) {
        // Construct intervals at each sweep line
        std::vector<Interval> obsSeg;
        std::vector<Interval> arenaSeg;

        const auto lineIdx = static_cast<Eigen::Index>(i);

        for (auto j = 0; j < intersect->arenaLow.cols(); ++j) {
            if (!std::isnan(intersect->arenaLow(lineIdx, j)) &&
                !std::isnan(intersect->arenaUpp(lineIdx, j))) {
                arenaSeg.push_back({intersect->arenaLow(lineIdx, j),
                                    intersect->arenaUpp(lineIdx, j)});
            }
        }
        for (auto j = 0; j < intersect->obstacleLow.cols(); ++j) {
            if (!std::isnan(intersect->obstacleLow(lineIdx, j)) &&
                !std::isnan(intersect->obstacleUpp(lineIdx, j))) {
                obsSeg.push_back({intersect->obstacleLow(lineIdx, j),
                                  intersect->obstacleUpp(lineIdx, j)});
            }
        }

        // Set operations for Collision-free intervals at each line
        std::vector<Interval> obsMerge = op.unions(obsSeg);
        std::vector<Interval> arenaIntersect = op.intersects(arenaSeg);
        interval[i] = op.complements(arenaIntersect, obsMerge);

        // x-(z-)coords
        std::vector<Coordinate> xL;
        std::vector<Coordinate> xU;
        std::vector<Coordinate> xM;

        for (Index j = 0; j < interval[i].size(); ++j) {
            xL.push_back(interval[i][j].s());
            xU.push_back(interval[i][j].e());
            xM.push_back((interval[i][j].s() + interval[i][j].e()) / 2.0);
        }
        freeLineSegment.xL.push_back(xL);
        freeLineSegment.xU.push_back(xU);
        freeLineSegment.xM.push_back(xM);
    }

    // Enhanced process to generate more valid vertices within free line
    // segement
    return enhanceDecomp(&freeLineSegment);
}

template <class RobotType, class ObjectType>
FreeSegment2D HighwayRoadMap<RobotType, ObjectType>::enhanceDecomp(
    const FreeSegment2D* current) {
    FreeSegment2D enhanced = *current;

    // Add new vertices within on sweep line
    for (size_t i = 0; i < current->ty.size() - 1; ++i) {
        for (size_t j1 = 0; j1 < current->xM[i].size(); ++j1) {
            for (size_t j2 = 0; j2 < current->xM[i + 1].size(); ++j2) {
                if (enhanced.xM[i][j1] < enhanced.xL[i + 1][j2] &&
                    enhanced.xU[i][j1] >= enhanced.xL[i + 1][j2]) {
                    enhanced.xU[i].push_back(enhanced.xL[i + 1][j2]);
                    enhanced.xL[i].push_back(enhanced.xL[i + 1][j2]);
                    enhanced.xM[i].push_back(enhanced.xL[i + 1][j2]);
                } else if (enhanced.xM[i][j1] > enhanced.xU[i + 1][j2] &&
                           enhanced.xL[i][j1] <= enhanced.xU[i + 1][j2]) {
                    enhanced.xU[i].push_back(enhanced.xU[i + 1][j2]);
                    enhanced.xL[i].push_back(enhanced.xU[i + 1][j2]);
                    enhanced.xM[i].push_back(enhanced.xU[i + 1][j2]);
                }

                if (enhanced.xM[i + 1][j2] < enhanced.xL[i][j1] &&
                    enhanced.xU[i + 1][j2] >= enhanced.xL[i][j1]) {
                    enhanced.xU[i + 1].push_back(enhanced.xL[i][j1]);
                    enhanced.xL[i + 1].push_back(enhanced.xL[i][j1]);
                    enhanced.xM[i + 1].push_back(enhanced.xL[i][j1]);
                } else if (enhanced.xM[i + 1][j2] > enhanced.xU[i][j1] &&
                           enhanced.xL[i + 1][j2] <= enhanced.xU[i][j1]) {
                    enhanced.xU[i + 1].push_back(enhanced.xU[i][j1]);
                    enhanced.xL[i + 1].push_back(enhanced.xU[i][j1]);
                    enhanced.xM[i + 1].push_back(enhanced.xU[i][j1]);
                }
            }
        }

        sort(enhanced.xL[i].begin(), enhanced.xL[i].end(),
             [](double a, double b) { return a < b; });
        sort(enhanced.xU[i].begin(), enhanced.xU[i].end(),
             [](double a, double b) { return a < b; });
        sort(enhanced.xM[i].begin(), enhanced.xM[i].end(),
             [](double a, double b) { return a < b; });
    }

    return enhanced;
}

template <class RobotType, class ObjectType>
void HighwayRoadMap<RobotType, ObjectType>::bridgeVertex(const Index idx1,
                                                         const Index idx2) {
    const auto v1 = res_.graph_structure.vertex.at(idx1);
    const auto v2 = res_.graph_structure.vertex.at(idx2);

    // Generate new bridge vertex
    auto vNew1 = v1;
    vNew1.at(2) = v2.at(2);
    auto vNew2 = v2;
    vNew2.at(2) = v1.at(2);

    auto vNew = v1;

    // Check validity of potential connections
    if (isSameLayerTransitionFree(v1, vNew1) &&
        isSameLayerTransitionFree(vNew1, v2)) {
        vNew = vNew1;
    } else if (isSameLayerTransitionFree(v1, vNew2) &&
               isSameLayerTransitionFree(vNew2, v2)) {
        vNew = vNew2;
    } else {
        return;
    }

    // Add new bridge vertex to graph is new connection is valid
    int idxNew = res_.graph_structure.vertex.size();
    res_.graph_structure.vertex.push_back(vNew);
    res_.graph_structure.edge.push_back(std::make_pair(idx1, idxNew));
    res_.graph_structure.weight.push_back(vectorEuclidean(v1, vNew));
    res_.graph_structure.edge.push_back(std::make_pair(idxNew, idx2));
    res_.graph_structure.weight.push_back(vectorEuclidean(vNew, v2));
}
