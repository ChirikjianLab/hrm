#include "planners/include/ompl_interface/OMPL3D.h"

OMPL3D::OMPL3D(const std::vector<Coordinate> &lowBound,
               const std::vector<Coordinate> &highBound, MultiBodyTree3D robot,
               const std::vector<SuperQuadrics> &arena,
               const std::vector<SuperQuadrics> &obs,
               const std::vector<Mesh> &obsMesh)
    : OMPLInterface<MultiBodyTree3D, SuperQuadrics>::OMPLInterface(robot, arena,
                                                                   obs),
      obsMesh_(obsMesh) {
    // Setup state space
    setStateSpace(lowBound, highBound);
}

OMPL3D::~OMPL3D() = default;

void OMPL3D::setup(const Index plannerId, const Index validStateSamplerId) {
    // Set collision checker
    ss_->setStateValidityChecker(
        [this](const ob::State *state) { return isStateValid(state); });
    setCollisionObject();

    // Set planner and sampler
    setPlanner(plannerId);
    setValidStateSampler(validStateSamplerId);

    ss_->setup();
}

bool OMPL3D::plan(const std::vector<Coordinate> &start,
                  const std::vector<Coordinate> &goal,
                  const double maxTimeInSec) {
    isSolved_ = false;
    if (!ss_) {
        return isSolved_;
    }

    // Set start and goal states for planning
    setStartAndGoalState(start, goal);

    // Path planning
    OMPL_INFORM("Planning...");

    try {
        ob::PlannerStatus solved = ss_->solve(maxTimeInSec);

        // Get solution status
        totalTime_ = ss_->getLastPlanComputationTime();

        if (solved && totalTime_ < maxTimeInSec) {
            // Number of nodes in solved path
            lengthPath_ = ss_->getSolutionPath().getStates().size();
            isSolved_ = true;
        }

    } catch (ompl::Exception &ex) {
        std::stringstream es;
        es << ex.what() << std::endl;
        OMPL_WARN(es.str().c_str());
    }

    // Get planning results
    getSolution();

    return true;
}

void OMPL3D::getSolution() {
    if (isSolved_) {
        try {
            const unsigned int INTERPOLATION_NUMBER = 200;

            // Get solution path
            ss_->simplifySolution();
            auto path = ss_->getSolutionPath();

            // Save interpolated path
            path.interpolate(INTERPOLATION_NUMBER);
            for (auto *state : path.getStates()) {
                path_.push_back(setVectorFromState(state));
            }
        } catch (ompl::Exception &ex) {
        }
    }

    // Retrieve planning data
    ob::PlannerData pd(ss_->getSpaceInformation());
    ss_->getPlannerData(pd);

    // Number of vertices, edges
    numGraphVertex_ = pd.numVertices();
    numGraphEdges_ = pd.numEdges();

    // Save vertices and edges
    const ob::State *state;
    vertex_.clear();
    for (unsigned int i = 0; i < numValidStates_; ++i) {
        state = pd.getVertex(i).getState()->as<ob::State>();
        vertex_.push_back(setVectorFromState(state));
    }

    std::vector<std::vector<unsigned int>> edgeInfo(numValidStates_);
    edge_.clear();
    for (unsigned int i = 0; i < numValidStates_; i++) {
        pd.getEdges(i, edgeInfo[i]);
        for (auto edgeI : edgeInfo[i]) {
            edge_.emplace_back(std::make_pair(i, edgeI));
        }
    }

    // Total number of checked and valid motions
    numCollisionChecks_ =
        pd.getSpaceInformation()->getMotionValidator()->getCheckedMotionCount();
    numValidStates_ =
        pd.getSpaceInformation()->getMotionValidator()->getValidMotionCount();

    validSpace_ = ss_->getSpaceInformation()->probabilityOfValidState(1000);
}

void OMPL3D::setStateSpace(const std::vector<Coordinate> &lowBound,
                           const std::vector<Coordinate> &highBound) {
    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, lowBound[0]);
    bounds.setLow(1, lowBound[1]);
    bounds.setLow(2, lowBound[2]);
    bounds.setHigh(0, highBound[0]);
    bounds.setHigh(1, highBound[1]);
    bounds.setHigh(2, highBound[2]);
    space->setBounds(bounds);

    ss_ = std::make_shared<og::SimpleSetup>(space);
}

void OMPL3D::setStateSampler(const Index stateSamplerId) {}

void OMPL3D::setStartAndGoalState(const std::vector<Coordinate> &start,
                                  const std::vector<Coordinate> &goal) {
    ob::ScopedState<ob::CompoundStateSpace> startState(ss_->getStateSpace());
    setStateFromVector(&start, &startState);
    startState.enforceBounds();

    ob::ScopedState<ob::CompoundStateSpace> goalState(ss_->getStateSpace());
    setStateFromVector(&goal, &goalState);
    goalState.enforceBounds();

    ss_->setStartAndGoalStates(startState, goalState);
}

void OMPL3D::setCollisionObject() {
    // Setup collision object for ellipsoidal robot parts
    GeometryPtr_t ellip(
        new fcl::Ellipsoidd(robot_.getBase().getSemiAxis().at(0),
                            robot_.getBase().getSemiAxis().at(1),
                            robot_.getBase().getSemiAxis().at(2)));
    objRobot_.emplace_back(fcl::CollisionObjectd(ellip));
    for (size_t i = 0; i < robot_.getNumLinks(); ++i) {
        GeometryPtr_t ellip(
            new fcl::Ellipsoidd(robot_.getLinks().at(i).getSemiAxis().at(0),
                                robot_.getLinks().at(i).getSemiAxis().at(1),
                                robot_.getLinks().at(i).getSemiAxis().at(2)));
        objRobot_.emplace_back(fcl::CollisionObjectd(ellip));
    }

    // Setup collision object for superquadric obstacles
    for (const auto &obstacle : obstacle_) {
        if (std::fabs(obstacle.getEpsilon().at(0) - 1.0) < 1e-6 &&
            std::fabs(obstacle.getEpsilon().at(1) - 1.0) < 1e-6) {
            GeometryPtr_t ellip(new fcl::Ellipsoidd(
                obstacle.getSemiAxis().at(0), obstacle.getSemiAxis().at(1),
                obstacle.getSemiAxis().at(2)));
            objObs_.emplace_back(fcl::CollisionObjectd(ellip));
        } else {
            objObs_.emplace_back(setCollisionObjectFromSQ(obstacle));
        }
    }
}

bool OMPL3D::isStateValid(const ob::State *state) const {
    return isSeparated(transformRobot(state));
}

// Get pose info and transform the robot
MultiBodyTree3D OMPL3D::transformRobot(const ob::State *state) const {
    std::vector<Coordinate> stateVar = setVectorFromState(state);

    SE3Transform tf;
    tf.topRightCorner(3, 1) =
        Eigen::Array3d({stateVar.at(0), stateVar.at(1), stateVar.at(2)});
    tf.topLeftCorner(3, 3) = Eigen::Quaterniond(stateVar.at(3), stateVar.at(4),
                                                stateVar.at(5), stateVar.at(6))
                                 .toRotationMatrix();
    tf.bottomRows(1) << 0.0, 0.0, 0.0, 1.0;

    MultiBodyTree3D robotAux = robot_;
    robotAux.robotTF(tf);

    return robotAux;
}

// Checking collision with obstacles
bool OMPL3D::isSeparated(const MultiBodyTree3D &robotAux) const {
    for (size_t i = 0; i < obstacle_.size(); ++i) {
        // For an ellipsoid and superquadrics, use FCL
        if (isCollision(robotAux.getBase(), objRobot_.at(0), obstacle_.at(i),
                        objObs_.at(i))) {
            return false;
        }

        for (size_t j = 0; j < robotAux.getNumLinks(); ++j) {
            if (isCollision(robotAux.getLinks().at(j), objRobot_.at(j + 1),
                            obstacle_.at(i), objObs_.at(i))) {
                return false;
            }
        }
    }

    return true;
}

bool OMPL3D::compareStates(const std::vector<Coordinate> &goalConfig,
                           const std::vector<Coordinate> &lastConfig) {
    bool res = true;
    for (size_t i = 0; i < 7; i++) {
        if (std::fabs(lastConfig[i] - goalConfig[i]) > 0.1) {
            res = false;
        }
    }
    return res;
}

void OMPL3D::saveVertexEdgeInfo(const std::string &filename_prefix) {
    ob::PlannerData pd(ss_->getSpaceInformation());

    // Write the output to .csv files
    std::ofstream file_state;
    std::vector<Coordinate> state;

    file_state.open(filename_prefix + "_state_3D.csv");
    for (unsigned int i = 0; i < pd.numVertices(); i++) {
        state = setVectorFromState(pd.getVertex(i).getState()->as<ob::State>());

        for (size_t j = 0; j < state.size(); ++j) {
            file_state << state[j];
            if (j == state.size() - 1) {
                file_state << '\n';
            } else {
                file_state << ',';
            }
        }
    }
    file_state.close();

    std::ofstream file_edge;
    file_edge.open(filename_prefix + "_edge_3D.csv");
    std::vector<std::vector<unsigned int>> edge(pd.numVertices());
    for (unsigned int i = 0; i < pd.numVertices(); i++) {
        pd.getEdges(i, edge[i]);
        for (unsigned int j = 0; j < edge[i].size(); j++) {
            file_edge << int(i) << " " << int(edge[i][j]) << "\n";
        }
    }
    file_edge.close();
}

void OMPL3D::savePathInfo(const std::string &filename_prefix) {
    const std::vector<ob::State *> &states = ss_->getSolutionPath().getStates();
    std::vector<Coordinate> state;

    std::ofstream file_traj;
    file_traj.open(filename_prefix + "_path_3D.csv");
    for (auto *omplState : states) {
        state = setVectorFromState(omplState->as<ob::State>());

        for (size_t j = 0; j < state.size(); ++j) {
            file_traj << state[j];
            if (j == state.size() - 1) {
                file_traj << '\n';
            } else {
                file_traj << ',';
            }
        }
    }
    file_traj.close();

    // Smooth path
    ss_->getSolutionPath().interpolate(50);
    const std::vector<ob::State *> &solutionStates =
        ss_->getSolutionPath().getStates();

    std::ofstream file_smooth_traj;
    file_smooth_traj.open(filename_prefix + "_smooth_path_3D.csv");
    for (auto *omplState : solutionStates) {
        state = setVectorFromState(omplState->as<ob::State>());

        for (size_t j = 0; j < state.size(); ++j) {
            file_smooth_traj << state[j];
            if (j == state.size() - 1) {
                file_smooth_traj << '\n';
            } else {
                file_smooth_traj << ',';
            }
        }
    }
    file_smooth_traj.close();
}

void OMPL3D::setStateFromVector(
    const std::vector<Coordinate> *stateVariables,
    ob::ScopedState<ob::CompoundStateSpace> *state) const {
    ob::ScopedState<ob::SE3StateSpace> stateTemp(ss_->getStateSpace());

    stateTemp->setXYZ(stateVariables->at(0), stateVariables->at(1),
                      stateVariables->at(2));
    stateTemp->rotation().w = stateVariables->at(3);
    stateTemp->rotation().x = stateVariables->at(4);
    stateTemp->rotation().y = stateVariables->at(5);
    stateTemp->rotation().z = stateVariables->at(6);
    stateTemp.enforceBounds();

    stateTemp >> *state;
}

std::vector<Coordinate> OMPL3D::setVectorFromState(
    const ob::State *state) const {
    std::vector<Coordinate> stateVariables(7, 0.0);

    // Store state in a vector
    stateVariables[0] = state->as<ob::SE3StateSpace::StateType>()->getX();
    stateVariables[1] = state->as<ob::SE3StateSpace::StateType>()->getY();
    stateVariables[2] = state->as<ob::SE3StateSpace::StateType>()->getZ();
    stateVariables[3] = state->as<ob::SE3StateSpace::StateType>()->rotation().w;
    stateVariables[4] = state->as<ob::SE3StateSpace::StateType>()->rotation().x;
    stateVariables[5] = state->as<ob::SE3StateSpace::StateType>()->rotation().y;
    stateVariables[6] = state->as<ob::SE3StateSpace::StateType>()->rotation().z;

    return stateVariables;
}
