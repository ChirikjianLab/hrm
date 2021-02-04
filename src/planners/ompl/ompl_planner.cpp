#include "planners/include/ompl/ompl_planner.h"

PlannerOMPL::PlannerOMPL(std::vector<double> lowBound,
                         std::vector<double> highBound,
                         const std::vector<SuperQuadrics> &robot,
                         const std::vector<SuperQuadrics> &arena,
                         const std::vector<SuperQuadrics> &obs,
                         const std::vector<Mesh> &obsMesh)
    : arena_(arena), robot_(robot), obstacles_(obs), obsMesh_(obsMesh) {
    // Setup state space
    setStateSpace(lowBound, highBound);
}

PlannerOMPL::~PlannerOMPL() {}

void PlannerOMPL::setup(const int plannerId, const int stateSamplerId,
                        const int validStateSamplerId) {
    // Set collision checker
    ss_->setStateValidityChecker(
        [this](const ob::State *state) { return isStateValid(state); });
    //    ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setCollisionObject();

    setPlanner(plannerId);
    setValidStateSampler(validStateSamplerId);

    ss_->setup();
    //        ss_->print();
}

bool PlannerOMPL::plan(const std::vector<double> &start,
                       const std::vector<double> &goal,
                       const double maxTimeInSec) {
    if (!ss_) {
        return false;
    }

    ob::ScopedState<> startState(ss_->getStateSpace());
    startState->as<ob::SE3StateSpace::StateType>()->setXYZ(start[0], start[1],
                                                           start[2]);
    startState->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
    startState->as<ob::SE3StateSpace::StateType>()->rotation().w = start[3];
    startState->as<ob::SE3StateSpace::StateType>()->rotation().x = start[4];
    startState->as<ob::SE3StateSpace::StateType>()->rotation().y = start[5];
    startState->as<ob::SE3StateSpace::StateType>()->rotation().z = start[6];

    ob::ScopedState<> goalState(ss_->getStateSpace());
    goalState->as<ob::SE3StateSpace::StateType>()->setXYZ(goal[0], goal[1],
                                                          goal[2]);
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().w = goal[3];
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().x = goal[4];
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().y = goal[5];
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().z = goal[6];

    startState.enforceBounds();
    goalState.enforceBounds();

    ss_->setStartAndGoalStates(startState, goalState);

    // Path planning
    OMPL_INFORM("Planning...");

    try {
        ob::PlannerStatus solved = ss_->solve(maxTimeInSec);
        totalTime_ = ss_->getLastPlanComputationTime();

        // Get solution status
        if (!solved || totalTime_ > maxTimeInSec) {
            isSolved_ = false;
        } else {
            // Number of nodes in solved path
            lengthPath_ = ss_->getSolutionPath().getStates().size();
            isSolved_ = true;
        }

    } catch (ompl::Exception &ex) {
        std::stringstream es;
        es << ex.what() << std::endl;
        OMPL_WARN(es.str().c_str());
    }

    // Get graph info
    ob::PlannerData pd(ss_->getSpaceInformation());
    ss_->getPlannerData(pd);

    // Number of nodes and edges in graph
    numGraphVertex_ = pd.numVertices();
    numGraphEdges_ = pd.numEdges();

    // Total number of checked and valid motions
    numCollisionChecks_ =
        pd.getSpaceInformation()->getMotionValidator()->getCheckedMotionCount();
    numValidStates_ =
        pd.getSpaceInformation()->getMotionValidator()->getValidMotionCount();

    validSpace_ = ss_->getSpaceInformation()->probabilityOfValidState(1000);

    return true;
}

void PlannerOMPL::getSolution() {}

void PlannerOMPL::setStateSpace(const std::vector<double> &lowBound,
                                const std::vector<double> &highBound) {
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

void PlannerOMPL::setPlanner(const int plannerId) {
    // Set planner
    if (plannerId == 0) {
        ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
    }
    if (plannerId == 1) {
        ss_->setPlanner(
            std::make_shared<og::LazyPRM>(ss_->getSpaceInformation()));
    }
    if (plannerId == 2) {
        ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
    }
    if (plannerId == 3) {
        ss_->setPlanner(
            std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
    }
    if (plannerId == 4) {
        ss_->setPlanner(std::make_shared<og::EST>(ss_->getSpaceInformation()));
    }
    if (plannerId == 5) {
        ss_->setPlanner(
            std::make_shared<og::KPIECE1>(ss_->getSpaceInformation()));
    }
}

void PlannerOMPL::setStateSampler(const int stateSamplerId) {}

// Set the valid state sampler
void PlannerOMPL::setValidStateSampler(const int validSamplerId) {
    if (validSamplerId == 0) {
        // Uniform sampler
        OMPL_INFORM("Using Uniform valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::UniformValidStateSampler>(si);
            });
    } else if (validSamplerId == 1) {
        // Gaussian sampler
        OMPL_INFORM("Using Gaussian valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::GaussianValidStateSampler>(si);
            });
    } else if (validSamplerId == 2) {
        // Obstacle-based sampler
        OMPL_INFORM("Using Obstacle-based valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
            });
    } else if (validSamplerId == 3) {
        // Maximum-clearance sampler
        OMPL_INFORM("Using Max-clearance valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                auto vss =
                    std::make_shared<ob::MaximizeClearanceValidStateSampler>(
                        si);
                vss->setNrImproveAttempts(5);
                return vss;
            });
    } else if (validSamplerId == 4) {
        // Bridge-test sampler
        OMPL_INFORM("Using Bridge-test valid state sampler");

        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
                return std::make_shared<ob::BridgeTestValidStateSampler>(si);
            });
    }
}

bool PlannerOMPL::isStateValid(const ob::State *state) const {
    for (unsigned int j = 0; j < robot_.size(); j++) {
        SuperQuadrics robotAux = robot_[j];
        robotAux.setPosition(
            {state->as<ob::SE3StateSpace::StateType>()->getX(),
             state->as<ob::SE3StateSpace::StateType>()->getY(),
             state->as<ob::SE3StateSpace::StateType>()->getZ()});

        robotAux.setQuaternion(Eigen::Quaterniond(
            state->as<ob::SE3StateSpace::StateType>()->rotation().w,
            state->as<ob::SE3StateSpace::StateType>()->rotation().x,
            state->as<ob::SE3StateSpace::StateType>()->rotation().y,
            state->as<ob::SE3StateSpace::StateType>()->rotation().z));

        // Checking collision against obstacles
        for (unsigned int i = 0; i < obstacles_.size(); i++) {
            if (!checkSeparation(robot_[j], robotAux, objRobot_[j],
                                 obstacles_[i], objObs_[i])) {
                return false;
            }
        }
    }

    return true;
}

// Returns true when separated and false when overlapping
bool PlannerOMPL::checkSeparation(const SuperQuadrics &robotOrigin,
                                  const SuperQuadrics &robotAux,
                                  fcl::CollisionObject<double> objE,
                                  const SuperQuadrics &obs,
                                  fcl::CollisionObject<double> objSQ) const {
    /*
     * \brief Ellipsoid object
     * Needs to transform each part according to its relative pose with the
     * base
     */
    objE.setRotation(robotAux.getQuaternion().toRotationMatrix() *
                     robotOrigin.getQuaternion().toRotationMatrix());
    objE.setTranslation(fcl::Vector3d(robotAux.getPosition().at(0),
                                      robotAux.getPosition().at(1),
                                      robotAux.getPosition().at(2)) +
                        robotAux.getQuaternion().toRotationMatrix() *
                            fcl::Vector3d(robotOrigin.getPosition().at(0),
                                          robotOrigin.getPosition().at(1),
                                          robotOrigin.getPosition().at(2)));

    // Mesh object for SQ
    objSQ.setRotation(obs.getQuaternion().toRotationMatrix());
    objSQ.setTranslation(fcl::Vector3d(obs.getPosition().at(0),
                                       obs.getPosition().at(1),
                                       obs.getPosition().at(2)));

    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;

    fcl::collide(&objE, &objSQ, request, result);
    return !result.isCollision();
}

bool PlannerOMPL::compareStates(std::vector<double> goal_config,
                                std::vector<double> last_config) {
    bool res = true;
    for (size_t i = 0; i < 7; i++) {
        if (std::fabs(last_config[i] - goal_config[i]) > 0.1) {
            res = false;
        }
    }
    return res;
}

void PlannerOMPL::setCollisionObject() {
    for (size_t i = 0; i < robot_.size(); i++) {
        GeometryPtr_t ellip(new fcl::Ellipsoidd(
            robot_.at(i).getSemiAxis().at(0), robot_.at(i).getSemiAxis().at(1),
            robot_.at(i).getSemiAxis().at(2)));
        objRobot_.push_back(fcl::CollisionObjectd(ellip));
    }

    for (size_t i = 0; i < obstacles_.size(); i++) {
        if (std::fabs(obstacles_.at(i).getEpsilon().at(0) - 1.0) < 1e-6 &&
            std::fabs(obstacles_.at(i).getEpsilon().at(1) - 1.0) < 1e-6) {
            GeometryPtr_t ellip(
                new fcl::Ellipsoidd(obstacles_.at(i).getSemiAxis().at(0),
                                    obstacles_.at(i).getSemiAxis().at(1),
                                    obstacles_.at(i).getSemiAxis().at(2)));
            objObs_.push_back(fcl::CollisionObjectd(ellip));
        } else {
            fcl::BVHModel<fcl::OBBRSS<double>> *model_sq =
                new fcl::BVHModel<fcl::OBBRSS<double>>();
            model_sq->beginModel();
            model_sq->addSubModel(obsMesh_[i].vertices, obsMesh_[i].triangles);
            model_sq->endModel();
            objObs_.push_back(fcl::CollisionObjectd(GeometryPtr_t(model_sq)));
        }
    }
}

void PlannerOMPL::getVertexEdgeInfo() {
    ob::PlannerData pd(ss_->getSpaceInformation());

    // Write the output to .csv files
    std::ofstream file_state;
    const ob::State *state;
    file_state.open("ompl_state_3D.csv");
    for (unsigned int i = 0; i < pd.numVertices(); i++) {
        state = pd.getVertex(i).getState()->as<ob::State>();
        file_state << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->rotation().w
                   << ","
                   << state->as<ob::SE3StateSpace::StateType>()->rotation().x
                   << ","
                   << state->as<ob::SE3StateSpace::StateType>()->rotation().y
                   << ","
                   << state->as<ob::SE3StateSpace::StateType>()->rotation().z
                   << "\n";
    }
    file_state.close();

    std::ofstream file_edge;
    file_edge.open("ompl_edge_3D.csv");
    std::vector<std::vector<unsigned int>> edge(pd.numVertices());
    for (unsigned int i = 0; i < pd.numVertices(); i++) {
        pd.getEdges(i, edge[i]);
        for (unsigned int j = 0; j < edge[i].size(); j++)
            file_edge << int(i) << " " << int(edge[i][j]) << "\n";
    }
    file_edge.close();
}

void PlannerOMPL::getPathInfo() {
    const std::vector<ob::State *> &states = ss_->getSolutionPath().getStates();
    ob::State *state;

    std::ofstream file_traj;
    file_traj.open("ompl_path_3D.csv");
    for (size_t i = 0; i < states.size(); ++i) {
        state = states[i]->as<ob::State>();
        file_traj << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
                  << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
                  << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
                  << state->as<ob::SE3StateSpace::StateType>()->rotation().w
                  << ","
                  << state->as<ob::SE3StateSpace::StateType>()->rotation().x
                  << ","
                  << state->as<ob::SE3StateSpace::StateType>()->rotation().y
                  << ","
                  << state->as<ob::SE3StateSpace::StateType>()->rotation().z
                  << "\n";
    }
    file_traj.close();

    // Smooth path
    ss_->getSolutionPath().interpolate(50);
    const std::vector<ob::State *> &s_states =
        ss_->getSolutionPath().getStates();

    std::ofstream file_smooth_traj;
    file_smooth_traj.open("ompl_smooth_path_3D.csv");
    for (size_t i = 0; i < s_states.size(); ++i) {
        state = s_states[i]->as<ob::State>();
        file_smooth_traj
            << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
            << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
            << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
            << state->as<ob::SE3StateSpace::StateType>()->rotation().w << ","
            << state->as<ob::SE3StateSpace::StateType>()->rotation().x << ","
            << state->as<ob::SE3StateSpace::StateType>()->rotation().y << ","
            << state->as<ob::SE3StateSpace::StateType>()->rotation().z << "\n";
    }
    file_smooth_traj.close();
}
