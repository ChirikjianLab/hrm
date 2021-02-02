#include "planners/include/ompl/PlannerOMPL.h"

#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

ob::ValidStateSamplerPtr allocUniformStateSampler(
    const ob::SpaceInformation *si) {
    return std::make_shared<ob::UniformValidStateSampler>(si);
}

// return an obstacle-based sampler
ob::ValidStateSamplerPtr allocOBValidStateSampler(
    const ob::SpaceInformation *si) {
    return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

// return a GaussianValidStateSampler
ob::ValidStateSamplerPtr allocGaussianValidStateSampler(
    const ob::SpaceInformation *si) {
    return std::make_shared<ob::GaussianValidStateSampler>(si);
}

// return a MaximizeClearanceValidStateSampler
ob::ValidStateSamplerPtr allocMaximizeClearanceValidStateSampler(
    const ob::SpaceInformation *si) {
    return std::make_shared<ob::MaximizeClearanceValidStateSampler>(si);
}

// return a BridgeTestValidStateSampler
ob::ValidStateSamplerPtr allocBridgeTestValidStateSampler(
    const ob::SpaceInformation *si) {
    return std::make_shared<ob::BridgeTestValidStateSampler>(si);
}

PlannerOMPL::PlannerOMPL(std::vector<double> lowBound,
                         std::vector<double> highBound,
                         const std::vector<SuperQuadrics> &robot,
                         const std::vector<SuperQuadrics> &arena,
                         const std::vector<SuperQuadrics> &obs,
                         const std::vector<Mesh> &obsMesh, const int planner,
                         const int sampler)
    : lowBound_(lowBound),
      highBound_(highBound),
      arena_(arena),
      robot_(robot),
      obstacles_(obs),
      obsMesh_(obsMesh),
      planner_(planner),
      sampler_(sampler) {
    // Initiate object for collision detection using FCL
    setCollisionObj();

    // Set state space and bounds
    setStateSpace();

    // Set planner
    if (planner_ == 0) {
        ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
    }
    if (planner_ == 1) {
        ss_->setPlanner(
            std::make_shared<og::LazyPRM>(ss_->getSpaceInformation()));
    }
    if (planner_ == 2) {
        ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
    }
    if (planner_ == 3) {
        ss_->setPlanner(
            std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
    }
    if (planner_ == 4) {
        ss_->setPlanner(std::make_shared<og::EST>(ss_->getSpaceInformation()));
    }
    if (planner_ == 5) {
        ss_->setPlanner(
            std::make_shared<og::KPIECE1>(ss_->getSpaceInformation()));
    }

    // Setting up the sampler
    if (sampler_ == 0) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocUniformStateSampler);
    }
    if (sampler_ == 1) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocOBValidStateSampler);
    }
    if (sampler_ == 2) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocGaussianValidStateSampler);
    }
    if (sampler_ == 3) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocMaximizeClearanceValidStateSampler);
    }
    if (sampler_ == 4) {
        ss_->getSpaceInformation()->setValidStateSamplerAllocator(
            allocBridgeTestValidStateSampler);
    }
}

PlannerOMPL::~PlannerOMPL() {}

bool PlannerOMPL::plan(const std::vector<double> &start,
                       const std::vector<double> &goal) {
    if (!ss_) {
        return false;
    }

    // Set start and goal states
    start_ = start;
    goal_ = goal;

    setStartAndGoalStates();

    // Path planning
    std::cout << "Planning..." << std::endl;

    try {
        ob::PlannerStatus solved = ss_->solve(maxPlanningTime_);
        totalTime = ss_->getLastPlanComputationTime();

        // Get solution status
        if (!solved || totalTime > maxPlanningTime_) {
            flag = false;
        } else {
            // Number of nodes in solved path
            numPathNodes = ss_->getSolutionPath().getStates().size();
            flag = true;
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
    numGraphNodes = pd.numVertices();
    numGraphEdges = pd.numEdges();

    // Total number of checked and valid motions
    numCheckedNodes =
        pd.getSpaceInformation()->getMotionValidator()->getCheckedMotionCount();
    numValidNodes =
        pd.getSpaceInformation()->getMotionValidator()->getValidMotionCount();

    validSpace = ss_->getSpaceInformation()->probabilityOfValidState(1000);

    return true;
}

void PlannerOMPL::setStateSpace() {
    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, lowBound_[0]);
    bounds.setLow(1, lowBound_[1]);
    bounds.setLow(2, lowBound_[2]);
    bounds.setHigh(0, highBound_[0]);
    bounds.setHigh(1, highBound_[1]);
    bounds.setHigh(2, highBound_[2]);
    space->setBounds(bounds);

    ss_ = std::make_shared<og::SimpleSetup>(space);
    space->setup();

    ss_->setStateValidityChecker(
        [this](const ob::State *state) { return isStateValid(state); });

    // ss_->getSpaceInformation()->setStateValidityCheckingResolution(1/
    // space->getMaximumExtent());
}

void PlannerOMPL::setStartAndGoalStates() {
    ob::ScopedState<> startState(ss_->getStateSpace());
    startState->as<ob::SE3StateSpace::StateType>()->setXYZ(start_[0], start_[1],
                                                           start_[2]);
    startState->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
    startState->as<ob::SE3StateSpace::StateType>()->rotation().w = start_[3];
    startState->as<ob::SE3StateSpace::StateType>()->rotation().x = start_[4];
    startState->as<ob::SE3StateSpace::StateType>()->rotation().y = start_[5];
    startState->as<ob::SE3StateSpace::StateType>()->rotation().z = start_[6];

    ob::ScopedState<> goalState(ss_->getStateSpace());
    goalState->as<ob::SE3StateSpace::StateType>()->setXYZ(goal_[0], goal_[1],
                                                          goal_[2]);
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().w = goal_[3];
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().x = goal_[4];
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().y = goal_[5];
    goalState->as<ob::SE3StateSpace::StateType>()->rotation().z = goal_[6];

    startState.enforceBounds();
    goalState.enforceBounds();

    ss_->setStartAndGoalStates(startState, goalState);
    ss_->setup();
    //        ss_->print();
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

void PlannerOMPL::setCollisionObj() {
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
