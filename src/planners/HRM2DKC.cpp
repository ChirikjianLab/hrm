#include "hrm/planners/HRM2DKC.h"

hrm::planners::HRM2DKC::HRM2DKC(const MultiBodyTree2D& robot,
                                const std::vector<SuperEllipse>& arena,
                                const std::vector<SuperEllipse>& obs,
                                const PlanningRequest& req)
    : HRM2D(robot, arena, obs, req) {
    // Enlarge the robot
    SuperEllipse baseInflated = robot_.getBase();
    baseInflated.setSemiAxis(
        {baseInflated.getSemiAxis().at(0) * (1 + inflationFactor_),
         baseInflated.getSemiAxis().at(1) * (1 + inflationFactor_)});
    MultiBodyTree2D robot_infla(baseInflated);

    // Update robot in the free space computator
    freeSpacePtr_->setRobot(robot_infla);
    freeSpacePtr_->setup(param_.numLineY, param_.boundaryLimits[0],
                         param_.boundaryLimits[1]);
}

hrm::planners::HRM2DKC::~HRM2DKC() = default;

void hrm::planners::HRM2DKC::connectMultiSlice() {
    Index n = res_.graphStructure.vertex.size();
    Index n11;
    Index n12;
    Index n2;
    Index start = 0;

    std::vector<Coordinate> v1;
    std::vector<Coordinate> v2;
    std::vector<Coordinate> midVtx;

    for (size_t i = 0; i < param_.numSlice; ++i) {
        // Find vertex only in adjecent slices
        n11 = vertexIdx_.at(i).slice;
        if (i != param_.numSlice - 1) {
            n2 = vertexIdx_.at(i + 1).slice;
            n12 = n11;
        } else {
            n2 = vertexIdx_.at(0).slice;
            n12 = 0;
        }

        // Nearest vertex btw slices
        for (size_t m = start; m < n11; ++m) {
            v1 = res_.graphStructure.vertex[m];

            for (size_t m2 = n12; m2 < n2; ++m2) {
                v2 = res_.graphStructure.vertex[m2];

                // Judge connectivity using Kinematics of Containment
                midVtx = addMiddleVertex(v1, v2);
                if (!midVtx.empty()) {
                    res_.graphStructure.vertex.push_back(midVtx);

                    res_.graphStructure.edge.push_back(std::make_pair(m, n));
                    res_.graphStructure.weight.push_back(
                        vectorEuclidean(v1, midVtx));
                    res_.graphStructure.edge.push_back(std::make_pair(m2, n));
                    res_.graphStructure.weight.push_back(
                        vectorEuclidean(v2, midVtx));
                    n++;
                    break;
                }
            }
            midVtx.clear();
        }
        start = n11;
    }
}

std::vector<hrm::Coordinate> hrm::planners::HRM2DKC::addMiddleVertex(
    std::vector<Coordinate> vtx1, std::vector<Coordinate> vtx2) {
    // Connect vertexes among different slices, and add a bridge vertex to the
    // roadmap
    std::vector<Coordinate> midVtx;
    std::vector<Coordinate> pt;
    std::vector<Coordinate> pt1;
    std::vector<Coordinate> pt2;
    bool flag;

    for (size_t iter = 0; iter < param_.numPoint; iter++) {
        pt.clear();
        pt1.clear();
        pt2.clear();
        for (size_t i = 0; i < vtx1.size(); ++i) {
            pt.push_back((rand() * (vtx1[i] - vtx2[i])) / RAND_MAX + vtx2[i]);
            pt1.push_back(pt[i] - vtx1[i]);
            pt2.push_back(pt[i] - vtx2[i]);
        }
        flag = isPtInPoly(polyhedronVertex_, pt1);
        if (flag) {
            flag = isPtInPoly(polyhedronVertex_, pt2);
            if (flag) {
                midVtx = pt;
                return midVtx;
            }
        }
    }

    return midVtx;
}
