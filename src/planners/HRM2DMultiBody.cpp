#include "include/HRM2DMultiBody.h"

#define pi 3.1415926

HRM2DMultiBody::HRM2DMultiBody(const MultiBodyTree2D& robot,
                               const std::vector<SuperEllipse>& arena,
                               const std::vector<SuperEllipse>& obs,
                               const PlanningRequest& req)
    : HighwayRoadMap2D(robot.getBase(), arena, obs, req), RobotM_(robot) {}

HRM2DMultiBody::~HRM2DMultiBody() {}

void HRM2DMultiBody::buildRoadmap() {
    // angle steps
    double dr = 2 * pi / (param_.NUM_LAYER - 1);

    // Setup rotation angles: angle range [-pi,pi]
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        ang_r.push_back(-pi + dr * i);
    }

    // Get the current Transformation
    Eigen::Matrix3d tf;
    tf.setIdentity();

    // Construct roadmap
    for (size_t i = 0; i < param_.NUM_LAYER; ++i) {
        // Set rotation matrix to robot
        tf.topLeftCorner(2, 2) =
            Eigen::Rotation2Dd(ang_r.at(i)).toRotationMatrix();
        RobotM_.robotTF(tf);
        robot_.setAngle(ang_r.at(i));

        Boundary bd = boundaryGen();
        FreeSegment2D freeSeg = sweepLine2D(&bd);
        connectOneLayer2D(&freeSeg);
        N_v_layer.push_back(res_.graph_structure.vertex.size());
    }

    // Connect adjacent layers using middle C-layer
    connectMultiLayer();
}

Boundary HRM2DMultiBody::boundaryGen() {
    Boundary bd;

    // Minkowski boundary points
    std::vector<Eigen::MatrixXd> bd_aux;
    for (size_t i = 0; i < size_t(N_s); ++i) {
        bd_aux = RobotM_.minkSum(arena_.at(i), -1);
        for (size_t j = 0; j < bd_aux.size(); ++j) {
            bd.arena.push_back(bd_aux.at(j));
        }
        bd_aux.clear();
    }
    for (size_t i = 0; i < size_t(N_o); ++i) {
        bd_aux = RobotM_.minkSum(obs_.at(i), 1);
        for (size_t j = 0; j < bd_aux.size(); ++j) {
            bd.obstacle.push_back(bd_aux.at(j));
        }
        bd_aux.clear();
    }

    return bd;
}

bool HRM2DMultiBody::isMultiLayerTransitionFree(const std::vector<double>& v1,
                                                const std::vector<double>& v2) {
    double dt = 1.0 / (param_.NUM_POINT - 1);
    for (size_t i = 0; i < param_.NUM_POINT; ++i) {
        // Interpolated robot motion from V1 to V2
        std::vector<double> vStep;
        for (size_t j = 0; j < v1.size(); ++j) {
            vStep.push_back((1.0 - i * dt) * v1[j] + i * dt * v2[j]);
        }

        // Transform the robot
        setTransform(vStep);

        // Base: determine whether each step is within CF-Line of bridgeLayer
        if (!isPtInCFLine(freeSeg_[0], RobotM_.getBase().getPosition())) {
            return false;
        }

        // For each link, check whether its center is within CF-segment of
        // bridgeLayer
        for (size_t j = 0; j < RobotM_.getNumLinks(); ++j) {
            if (!isPtInCFLine(freeSeg_[j + 1],
                              RobotM_.getLinks()[j].getPosition())) {
                return false;
            }
        }
    }

    return true;
}

bool HRM2DMultiBody::isPtInCFLine(const FreeSegment2D& freeSeg,
                                  const std::vector<double>& V) {
    std::vector<bool> isInLine(2, false);

    for (size_t i = 1; i < freeSeg.ty.size(); ++i) {
        // Locate to the sweep line of the vertex
        if (freeSeg.ty[i] < V[1]) {
            continue;
        }

        // z-coordinate within the current line
        for (size_t j = 0; j < freeSeg.xM[i].size(); ++j) {
            if ((V[0] >= freeSeg.xL[i][j]) && (V[0] <= freeSeg.xU[i][j])) {
                isInLine[0] = true;
                break;
            }
        }

        // z-coordinate within the adjacent line
        for (size_t j = 0; j < freeSeg.xM[i - 1].size(); ++j) {
            if ((V[0] >= freeSeg.xL[i - 1][j]) &&
                (V[0] <= freeSeg.xU[i - 1][j])) {
                isInLine[1] = true;
                break;
            }
        }
    }

    // z-coordinate within all neighboring sweep lines, then collision free
    if (isInLine[0] && isInLine[1]) {
        return true;
    } else {
        return false;
    }
}

void HRM2DMultiBody::computeTFE(const double thetaA, const double thetaB,
                                std::vector<SuperEllipse>* tfe) {
    tfe->clear();

    // Compute a tightly-fitted ellipse that bounds rotational motions from
    // thetaA to thetaB
    tfe->push_back(getTFE2D(RobotM_.getBase().getSemiAxis(), thetaA, thetaB,
                            uint(param_.NUM_POINT),
                            RobotM_.getBase().getNum()));

    for (size_t i = 0; i < RobotM_.getNumLinks(); ++i) {
        Eigen::Rotation2Dd rotLink(
            Eigen::Matrix2d(RobotM_.getTF().at(i).topLeftCorner(2, 2)));
        Eigen::Rotation2Dd rotA(Eigen::Rotation2Dd(thetaA).toRotationMatrix() *
                                rotLink);
        Eigen::Rotation2Dd rotB(Eigen::Rotation2Dd(thetaB).toRotationMatrix() *
                                rotLink);

        tfe->push_back(getTFE2D(
            RobotM_.getLinks().at(i).getSemiAxis(), rotA.angle(), rotB.angle(),
            uint(param_.NUM_POINT), RobotM_.getLinks().at(i).getNum()));
    }
}

void HRM2DMultiBody::setTransform(const std::vector<double>& v) {
    Eigen::Matrix3d g;
    g.topLeftCorner(2, 2) = Eigen::Rotation2Dd(v[2]).toRotationMatrix();
    g.topRightCorner(2, 1) = Eigen::Vector2d(v[0], v[1]);
    g.bottomLeftCorner(1, 3) << 0, 0, 1;
    RobotM_.robotTF(g);
}
