#include "include/HRM2DKC.h"

hrm::planners::HRM2DKC::HRM2DKC(const MultiBodyTree2D& robot,
                                const std::vector<SuperEllipse>& arena,
                                const std::vector<SuperEllipse>& obs,
                                const PlanningRequest& req)
    : HRM2D(robot, arena, obs, req) {
    // Enlarge the robot
    SuperEllipse base_infla = robot_.getBase();
    base_infla.setSemiAxis(
        {base_infla.getSemiAxis().at(0) * (1 + inflationFactor_),
         base_infla.getSemiAxis().at(1) * (1 + inflationFactor_)});
    MultiBodyTree2D robot_infla(base_infla);

    // Update robot in the free space computator
    freeSpacePtr_->setRobot(robot_infla);
    freeSpacePtr_->setup(param_.numLineY, param_.boundaryLimits[0],
                         param_.boundaryLimits[1]);
}

hrm::planners::HRM2DKC::~HRM2DKC() = default;

void hrm::planners::HRM2DKC::connectMultiLayer() {
    Index n = res_.graphStructure.vertex.size();
    Index n_11;
    Index n_12;
    Index n_2;
    Index start = 0;

    std::vector<Coordinate> v1;
    std::vector<Coordinate> v2;
    std::vector<Coordinate> midVtx;

    for (size_t i = 0; i < param_.numLayer; ++i) {
        // Find vertex only in adjecent layers
        n_11 = vertexIdx_.at(i).layer;
        if (i != param_.numLayer - 1) {
            n_2 = vertexIdx_.at(i + 1).layer;
            n_12 = n_11;
        } else {
            n_2 = vertexIdx_.at(0).layer;
            n_12 = 0;
        }

        // Nearest vertex btw layers
        for (size_t m = start; m < n_11; ++m) {
            v1 = res_.graphStructure.vertex[m];

            for (size_t m2 = n_12; m2 < n_2; ++m2) {
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
        start = n_11;
    }
}

std::vector<hrm::Coordinate> hrm::planners::HRM2DKC::addMiddleVertex(
    std::vector<Coordinate> vtx1, std::vector<Coordinate> vtx2) {
    // Connect vertexes among different layers, and add a bridge vertex to the
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
