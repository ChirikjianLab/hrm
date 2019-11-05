#include "include/ompl_planner.h"

ob::ValidStateSamplerPtr allocUniformStateSampler(
    const ob::SpaceInformation *si) {
    return make_shared<ob::UniformValidStateSampler>(si);
}

// return an obstacle-based sampler
ob::ValidStateSamplerPtr allocOBValidStateSampler(
    const ob::SpaceInformation *si) {
    return make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

// return a GaussianValidStateSampler
ob::ValidStateSamplerPtr allocGaussianValidStateSampler(
    const ob::SpaceInformation *si) {
    return make_shared<ob::GaussianValidStateSampler>(si);
}

// return a MaximizeClearanceValidStateSampler
ob::ValidStateSamplerPtr allocMaximizeClearanceValidStateSampler(
    const ob::SpaceInformation *si) {
    return make_shared<ob::MaximizeClearanceValidStateSampler>(si);
}

// return a BridgeTestValidStateSampler
ob::ValidStateSamplerPtr allocBridgeTestValidStateSampler(
    const ob::SpaceInformation *si) {
    return make_shared<ob::BridgeTestValidStateSampler>(si);
}

ompl_planner::ompl_planner(vector<double> lowBound, vector<double> highBound,
                           vector<SuperQuadrics> robot_,
                           vector<SuperQuadrics> arena_,
                           vector<SuperQuadrics> obs_, vector<EMesh> obs_mesh_,
                           int planner, int sampler) {
    arena = arena_;
    robot = robot_;
    obstacles = obs_;
    obs_mesh = obs_mesh_;
    id_planner = planner;
    id_sampler = sampler;

    // Initiate object for collision detection using FCL
    setCollisionObj();

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

    ss_->setStateValidityChecker(
        [this](const ob::State *state) { return isStateValid(state); });
    space->setup();
    // ss_->getSpaceInformation()->setStateValidityCheckingResolution(1/
    // space->getMaximumExtent());

    // Set planner
    if (id_planner == 0) {
        ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
    }
    if (id_planner == 1) {
        ss_->setPlanner(
            std::make_shared<og::LazyPRM>(ss_->getSpaceInformation()));
    }
    if (id_planner == 2) {
        ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
    }
    if (id_planner == 3) {
        ss_->setPlanner(
            std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
    }
    if (id_planner == 4) {
        ss_->setPlanner(std::make_shared<og::EST>(ss_->getSpaceInformation()));
    }
    if (id_planner == 5) {
        ss_->setPlanner(
            std::make_shared<og::KPIECE1>(ss_->getSpaceInformation()));
    }

    // Setting up the sampler
    if (sampler == 0) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocUniformStateSampler);
    }
    if (sampler == 1) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocOBValidStateSampler);
    }
    if (sampler == 2) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocGaussianValidStateSampler);
    }
    if (sampler == 3) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocMaximizeClearanceValidStateSampler);
    }
    if (sampler == 4) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocBridgeTestValidStateSampler);
    }
}

bool ompl_planner::plan(std::vector<double> start_, std::vector<double> goal_) {
    if (!ss_) {
        return false;
    }

    ob::ScopedState<> start(ss_->getStateSpace());
    start->as<ob::SE3StateSpace::StateType>()->setXYZ(start_[0], start_[1],
                                                      start_[2]);
    start->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
    start->as<ob::SE3StateSpace::StateType>()->rotation().w = start_[3];
    start->as<ob::SE3StateSpace::StateType>()->rotation().x = start_[4];
    start->as<ob::SE3StateSpace::StateType>()->rotation().y = start_[5];
    start->as<ob::SE3StateSpace::StateType>()->rotation().z = start_[6];

    ob::ScopedState<> goal(ss_->getStateSpace());
    goal->as<ob::SE3StateSpace::StateType>()->setXYZ(goal_[0], goal_[1],
                                                     goal_[2]);
    goal->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
    goal->as<ob::SE3StateSpace::StateType>()->rotation().w = goal_[3];
    goal->as<ob::SE3StateSpace::StateType>()->rotation().x = goal_[4];
    goal->as<ob::SE3StateSpace::StateType>()->rotation().y = goal_[5];
    goal->as<ob::SE3StateSpace::StateType>()->rotation().z = goal_[6];

    start.enforceBounds();
    goal.enforceBounds();

    ss_->setStartAndGoalStates(start, goal);
    ss_->setup();
    //        ss_->print();

    // Path planning
    cout << "Planning..." << endl;
    ob::PlannerStatus solved = ss_->solve(60.0);
    if (!solved) {
        flag = false;
        return false;
    }

    totalTime = ss_->getLastPlanComputationTime();
    if (totalTime > 60.0) {
        flag = false;
        return false;
    }

    // Get graph info
    ob::PlannerData pd(ss_->getSpaceInformation());
    ss_->getPlannerData(pd);

    // Number of nodes and edges in graph
    numGraphNodes = pd.numVertices();
    numGraphEdges = pd.numEdges();

    // Total number of checked and valid motions
    numCheckedNodes =
        pd.getSpaceInformation()->getMotionValidator()->getCheckedMotionCount();
    numValidNodes =
        pd.getSpaceInformation()->getMotionValidator()->getValidMotionCount();

    // Number of nodes in solved path
    numPathNodes = ss_->getSolutionPath().getStates().size();
    validSpace = ss_->getSpaceInformation()->probabilityOfValidState(1000);

    return true;
}

bool ompl_planner::isStateValid(const ob::State *state) const {
    bool res = true;
    for (unsigned int j = 0; j < robot.size(); j++) {
        SuperQuadrics rob = robot[j];
        rob.setPosition({state->as<ob::SE3StateSpace::StateType>()->getX(),
                         state->as<ob::SE3StateSpace::StateType>()->getY(),
                         state->as<ob::SE3StateSpace::StateType>()->getZ()});

        rob.setQuaternion(Quaterniond(
            state->as<ob::SE3StateSpace::StateType>()->rotation().w,
            state->as<ob::SE3StateSpace::StateType>()->rotation().x,
            state->as<ob::SE3StateSpace::StateType>()->rotation().y,
            state->as<ob::SE3StateSpace::StateType>()->rotation().z));

        // Checking collision against obstacles
        for (unsigned int i = 0; i < obstacles.size(); i++) {
            bool aux = checkSeparation(rob, robot[j], obj_robot[j],
                                       obstacles[i], obj_obs[i]);
            if (!aux) return false;
        }
    }
    return res;
}

// Returns true when separated and false when overlapping
bool ompl_planner::checkSeparation(SuperQuadrics robot, SuperQuadrics r_,
                                   CollisionObject<double> obj_ellip,
                                   SuperQuadrics obs,
                                   CollisionObject<double> obj_sq) const {
    // Ellipsoid object
    obj_ellip.setRotation(robot.getQuaternion().toRotationMatrix() *
                          r_.getQuaternion().toRotationMatrix());
    obj_ellip.setTranslation(
        fcl::Vector3d(robot.getPosition().at(0), robot.getPosition().at(1),
                      robot.getPosition().at(2)) +
        robot.getQuaternion().toRotationMatrix() *
            fcl::Vector3d(r_.getPosition().at(0), r_.getPosition().at(1),
                          r_.getPosition().at(2)));

    // Mesh object for SQ
    obj_sq.setRotation(obs.getQuaternion().toRotationMatrix());
    obj_sq.setTranslation(fcl::Vector3d(obs.getPosition().at(0),
                                        obs.getPosition().at(1),
                                        obs.getPosition().at(2)));

    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;

    fcl::collide(&obj_ellip, &obj_sq, request, result);
    return !result.isCollision();
}

bool ompl_planner::compareStates(vector<double> goal_config,
                                 vector<double> last_config) {
    bool res = true;
    for (size_t i = 0; i < 7; i++) {
        if (std::fabs(last_config[i] - goal_config[i]) > 0.1) {
            res = false;
        }
    }
    return res;
}

void ompl_planner::setCollisionObj() {
    for (size_t i = 0; i < robot.size(); i++) {
        GeometryPtr_t ellip(new fcl::Ellipsoidd(
            robot.at(i).getSemiAxis().at(0), robot.at(i).getSemiAxis().at(1),
            robot.at(i).getSemiAxis().at(2)));
        obj_robot.push_back(fcl::CollisionObjectd(ellip));
    }

    for (size_t i = 0; i < obstacles.size(); i++) {
        if (std::fabs(obstacles.at(i).getEpsilon().at(0) - 1.0) < 1e-6 &&
            std::fabs(obstacles.at(i).getEpsilon().at(1) - 1.0) < 1e-6) {
            GeometryPtr_t ellip(
                new fcl::Ellipsoidd(obstacles.at(i).getSemiAxis().at(0),
                                    obstacles.at(i).getSemiAxis().at(1),
                                    obstacles.at(i).getSemiAxis().at(2)));
            obj_obs.push_back(fcl::CollisionObjectd(ellip));
        } else {
            fcl::BVHModel<fcl::OBBRSS<double>> *model_sq =
                new fcl::BVHModel<fcl::OBBRSS<double>>();
            model_sq->beginModel();
            model_sq->addSubModel(obs_mesh[i].vertices, obs_mesh[i].triangles);
            model_sq->endModel();
            obj_obs.push_back(fcl::CollisionObjectd(GeometryPtr_t(model_sq)));
        }
    }
}

void ompl_planner::getVertexEdgeInfo() {
    ob::PlannerData pd(ss_->getSpaceInformation());

    // Write the output to .csv files
    ofstream file_state;
    const ob::State *state;
    file_state.open("ompl_state3d.csv");
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

    ofstream file_edge;
    file_edge.open("ompl_edge3d.csv");
    vector<vector<unsigned int>> edge(pd.numVertices());
    for (unsigned int i = 0; i < pd.numVertices(); i++) {
        pd.getEdges(i, edge[i]);
        for (unsigned int j = 0; j < edge[i].size(); j++)
            file_edge << int(i) << " " << int(edge[i][j]) << "\n";
    }
    file_edge.close();
}

void ompl_planner::getPathInfo() {
    const vector<ob::State *> &states = ss_->getSolutionPath().getStates();
    ob::State *state;

    ofstream file_traj;
    file_traj.open("ompl_path3d.csv");
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
    const vector<ob::State *> &s_states = ss_->getSolutionPath().getStates();

    ofstream file_smooth_traj;
    file_smooth_traj.open("ompl_smooth_path3d.csv");
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

ompl_planner::~ompl_planner() {}
