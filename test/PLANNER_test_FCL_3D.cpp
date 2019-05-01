#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/geometric/planners/prm/PRMstar.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>

//#include <fcl/fcl.h>
//#include <fcl/geometry/shape/ellipsoid.h>
//#include <fcl/narrowphase/collision_object.h>
//#include <fcl/narrowphase/collision.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <string>

#include <src/geometry/superquadrics.h>
#include <include/parse2dcsvfile.h>
#include <include/mesh_gen.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;
using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

EMesh getMesh(SuperQuadrics sq){
    sq.Shape.q.setIdentity();
    sq.Shape.pos[0] = 0.0; sq.Shape.pos[1] = 0.0; sq.Shape.pos[2] = 0.0;

    EMesh M;
//    MeshGenerator MeshGen;
//    ParametricPoints pts = MeshGen.getBoundary3D(sq);
//    M = MeshGen.getMesh(pts);

    long n = sq.n;
    long Num = (n-1)*(n-1);

    for(int i=0; i<sq.num; i++) M.vertices.push_back({sq.originShape()(0,i),
                                                      sq.originShape()(1,i),
                                                      sq.originShape()(2,i)});

    ArrayXd q((n-1)*(n-1));
    for(int i=0; i<n-1; i++) q.segment(i*(n-1),(n-1)) = ArrayXd::LinSpaced(n-1,i*n,(i+1)*n-2);

    MatrixXd faces = MatrixXd::Zero(2*Num,3);
    faces.block(0,0,Num,1) = q;
    faces.block(0,1,Num,1) = q + n;
    faces.block(0,2,Num,1) = q + n + 1;
    faces.block(Num,0,Num,1) = q;
    faces.block(Num,1,Num,1) = q + 1;
    faces.block(Num,2,Num,1) = q + n + 1;

    for(int i=0; i<faces.rows(); i++) M.triangles.push_back({size_t(faces(i,0)),size_t(faces(i,1)),size_t(faces(i,2))});

    return M;
}

vector<SuperQuadrics> generateSQ(string file_name, double D){
    // Read config file
    inputFile file;
    vector<vector<double>> config = file.parse2DCsvFile(file_name);

    // Generate SQ object
    vector<SuperQuadrics> obj(config.size());
    for(size_t j = 0; j < config.size(); j++){
        obj[j].Shape.a[0] = config[j][0];
        obj[j].Shape.a[1] = config[j][1];
        obj[j].Shape.a[2] = config[j][2];
        obj[j].Shape.eps[0] = config[j][3];
        obj[j].Shape.eps[1] = config[j][4];
        obj[j].Shape.pos[0] = config[j][5];
        obj[j].Shape.pos[1] = config[j][6];
        obj[j].Shape.pos[2] = config[j][7];
        obj[j].Shape.q.w() = config[j][8];
        obj[j].Shape.q.x() = config[j][9];
        obj[j].Shape.q.y() = config[j][10];
        obj[j].Shape.q.z() = config[j][11];
        obj[j].n = long(D);
    }

    return obj;
}

class FCLtester3D{
public:
    typedef Matrix<double,4,1> Vector4d;

    og::SimpleSetupPtr ss_;
    vector<SuperQuadrics> arena;
    vector<SuperQuadrics> robot;
    vector<SuperQuadrics> obstacles;
    vector<EMesh> obs_mesh;

    vector< fcl::CollisionObject<double> > obj_robot, obj_obs;

    unsigned int nodes_graph;
    unsigned int edges_graph;
    //    int total_random;
    unsigned long nodes_path;
    int flag;
//    double astar_time;
//    double construct_time;
    double total_time;
    double valid_space;
    int id_planner;

    FCLtester3D(vector<double> lowBound, vector<double> highBound,
                vector<SuperQuadrics> robot_,
                vector<SuperQuadrics> arena_, vector<SuperQuadrics> obs_, vector<EMesh> obs_mesh_, int id){

        arena = arena_;
        robot = robot_;
        obstacles = obs_;
        obs_mesh = obs_mesh_;
        id_planner = id;

        // Initiate object for collision detection using FCL
        setCollisionObj();

        auto space(std::make_shared<ob::SE3StateSpace>());
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0,lowBound[0]); bounds.setLow(1,lowBound[1]); bounds.setLow(2,lowBound[2]);
        bounds.setHigh(0,highBound[0]); bounds.setHigh(1,highBound[1]); bounds.setHigh(2,highBound[2]);
        space->setBounds(bounds);
        ss_ = std::make_shared<og::SimpleSetup>(space);

        ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
        space->setup();
//        ss_->getSpaceInformation()->setStateValidityCheckingResolution(1/ space->getMaximumExtent());

        // Set planner
        if(id_planner == 0) ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
        if(id_planner == 1) ss_->setPlanner(std::make_shared<og::PRMstar>(ss_->getSpaceInformation()));
        if(id_planner == 2) ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
        if(id_planner == 3) ss_->setPlanner(std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
    }

    bool compareStates(std::vector<double> goal_config, std::vector<double> last_config){
        bool res = true;
        for(size_t i=0; i<7; i++)
            if(abs(last_config[i] - goal_config[i]) > 0.1)
                res = false;
        return res;
    }

    bool plan(std::vector<double> start_, std::vector<double> goal_, int n){
        if (!ss_) return false;

        ob::ScopedState<> start(ss_->getStateSpace());
        start->as<ob::SE3StateSpace::StateType>()->setXYZ(start_[0],start_[1],start_[2]);
        start->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
        start->as<ob::SE3StateSpace::StateType>()->rotation().w = start_[3];
        start->as<ob::SE3StateSpace::StateType>()->rotation().x = start_[4];
        start->as<ob::SE3StateSpace::StateType>()->rotation().y = start_[5];
        start->as<ob::SE3StateSpace::StateType>()->rotation().z = start_[6];

        ob::ScopedState<> goal(ss_->getStateSpace());
        goal->as<ob::SE3StateSpace::StateType>()->setXYZ(goal_[0],goal_[1],goal_[2]);
        goal->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
        goal->as<ob::SE3StateSpace::StateType>()->rotation().w = goal_[3];
        goal->as<ob::SE3StateSpace::StateType>()->rotation().x = goal_[4];
        goal->as<ob::SE3StateSpace::StateType>()->rotation().y = goal_[5];
        goal->as<ob::SE3StateSpace::StateType>()->rotation().z = goal_[6];

        start.enforceBounds();
        goal.enforceBounds();

        ss_->setStartAndGoalStates(start, goal);
        ss_->setup();
        ss_->print();

        std::cout << "Planning..."<< std::endl;
        ob::PlannerStatus solved = ss_->solve(100);

        /*Getting times*/
        //flag = int(solved.operator bool());
        total_time = ss_->getLastPlanComputationTime();

        //Getting graph info
        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);
        nodes_graph = pd.numVertices();
        edges_graph = pd.numEdges();

        // Store all the valid states
        ofstream file_state;
        const ob::State *state;
        file_state.open("ompl_state3d.csv");
        for(unsigned int i=0; i<pd.numVertices(); i++){
            state = pd.getVertex(i).getState()->as<ob::State>();
            file_state << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
                       << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
                       << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
                       << state->as<ob::SE3StateSpace::StateType>()->rotation().w << ","
                       << state->as<ob::SE3StateSpace::StateType>()->rotation().x  << ","
                       << state->as<ob::SE3StateSpace::StateType>()->rotation().y  << ","
                       << state->as<ob::SE3StateSpace::StateType>()->rotation().z  << "\n";
        }
        file_state.close();

        // Store all the valid edges
        ofstream file_edge;
        file_edge.open("ompl_edge3d.csv");
        vector< vector<unsigned int> > edge(pd.numVertices());
        for(unsigned int i=0; i<pd.numVertices(); i++){
            pd.getEdges(i,edge[i]);
            for(unsigned int j=0; j<edge[i].size(); j++) file_edge << int(i) << " " << int(edge[i][j]) << "\n";
        }
        file_edge.close();

        nodes_path = ss_->getSolutionPath().getStates().size();
        valid_space = ss_->getSpaceInformation()->probabilityOfValidState(1000);

        bool aux_flag = false;
        if(solved){
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            ss_->getSolutionPath().print(std::cout);
            //Storing solution in a file
            const std::vector<ob::State*> &states = ss_->getSolutionPath().getStates();
            ob::State *state;
            ofstream file_traj;
            file_traj.open("ompl_path3d.csv");
            for( size_t i = 0 ; i < states.size( ) ; ++i ){
                state = states[i]->as<ob::State >( );
                file_traj << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
                          << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
                          << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
                          << state->as<ob::SE3StateSpace::StateType>()->rotation().w << ","
                          << state->as<ob::SE3StateSpace::StateType>()->rotation().x  << ","
                          << state->as<ob::SE3StateSpace::StateType>()->rotation().y  << ","
                          << state->as<ob::SE3StateSpace::StateType>()->rotation().z  << "\n";
            }
            file_traj.close();

            std::vector<double> final_conf;
            state = states.back()->as<ob::State >( );
            final_conf.resize(7);
            final_conf[0] = state->as<ob::SE3StateSpace::StateType>()->getX();
            final_conf[1] = state->as<ob::SE3StateSpace::StateType>()->getY();
            final_conf[2] = state->as<ob::SE3StateSpace::StateType>()->getZ();
            final_conf[3] = state->as<ob::SE3StateSpace::StateType>()->rotation().w;
            final_conf[4] = state->as<ob::SE3StateSpace::StateType>()->rotation().x;
            final_conf[5] = state->as<ob::SE3StateSpace::StateType>()->rotation().y;
            final_conf[6] = state->as<ob::SE3StateSpace::StateType>()->rotation().z;

            aux_flag = compareStates(goal_, final_conf);

            // Smooth path
            ss_->getSolutionPath().interpolate(50);

            // print the path to screen
            //ss_->getSolutionPath().printAsMatrix(std::cout);
            //Storing solution in a file
            const std::vector<ob::State*> &s_states = ss_->getSolutionPath().getStates();

            ofstream file_smooth_traj;
            file_smooth_traj.open("ompl_smooth_path3d.csv");
            for( size_t i = 0 ; i < s_states.size( ) ; ++i ){
                state = s_states[i]->as<ob::State >( );
                file_smooth_traj << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
                                 << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
                                 << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
                                 << state->as<ob::SE3StateSpace::StateType>()->rotation().w << ","
                                 << state->as<ob::SE3StateSpace::StateType>()->rotation().x  << ","
                                 << state->as<ob::SE3StateSpace::StateType>()->rotation().y  << ","
                                 << state->as<ob::SE3StateSpace::StateType>()->rotation().z  << "\n";
            }
            file_smooth_traj.close();
            flag = int(aux_flag);
            return true;
        }
        return false;
      }

    bool isStateValid(const ob::State *state) const{
        bool res = true;
        for(unsigned int j=0; j<robot.size();j++){
            SuperQuadrics::shape rob = robot[j].Shape;
            rob.pos[0] = state->as<ob::SE3StateSpace::StateType>()->getX();
            rob.pos[1] = state->as<ob::SE3StateSpace::StateType>()->getY();
            rob.pos[2] = state->as<ob::SE3StateSpace::StateType>()->getZ();

            rob.q = Quaterniond(state->as<ob::SE3StateSpace::StateType>()->rotation().w,
                                state->as<ob::SE3StateSpace::StateType>()->rotation().x,
                                state->as<ob::SE3StateSpace::StateType>()->rotation().y,
                                state->as<ob::SE3StateSpace::StateType>()->rotation().z);

            //Checking collision against obstacles
            for(unsigned int i=0; i<obstacles.size(); i++){
                bool aux = checkSeparation(rob, obj_robot[j], obstacles[i].Shape, obj_obs[i]);
                if(!aux) return false;
            }
        }
        return res;
    }

    //Returns true when separated and false when overlapping
    bool checkSeparation(SuperQuadrics::shape robot, fcl::CollisionObject<double> obj_ellip,
                         SuperQuadrics::shape obs, fcl::CollisionObject<double> obj_sq) const{
        // Ellipsoid object
        obj_ellip.setRotation(robot.q.toRotationMatrix());
        obj_ellip.setTranslation(fcl::Vector3d(robot.pos[0],robot.pos[1],robot.pos[2]));

        // Mesh object for SQ
        obj_sq.setRotation(obs.q.toRotationMatrix());
        obj_sq.setTranslation(fcl::Vector3d(obs.pos[0], obs.pos[1], obs.pos[2]));

        fcl::CollisionRequest<double> request;
        fcl::CollisionResult<double> result;

        fcl::collide(&obj_ellip, &obj_sq, request, result);
        return !result.isCollision();
    }

    void setCollisionObj(){
        for(size_t i=0; i<robot.size(); i++){
            GeometryPtr_t ellip(new fcl::Ellipsoidd(robot[i].Shape.a[0],robot[i].Shape.a[1],robot[i].Shape.a[2]));
            obj_robot.push_back(fcl::CollisionObjectd(ellip));
        }

        for(size_t i=0; i<obstacles.size(); i++){
            fcl::BVHModel<fcl::OBBRSS<double>>* model_sq = new fcl::BVHModel<fcl::OBBRSS<double>>();
            model_sq->beginModel();
            model_sq->addSubModel(obs_mesh[i].vertices, obs_mesh[i].triangles);
            model_sq->endModel();

            obj_obs.push_back(fcl::CollisionObjectd(GeometryPtr_t(model_sq)));
//            GeometryPtr_t ellip2(new fcl::Ellipsoidd(obstacles[i].Shape.a[0],obstacles[i].Shape.a[1],obstacles[i].Shape.a[2]));
//            obj_obs.push_back(fcl::CollisionObjectd(ellip2));
        }
    }
};

int main(int argc, char ** argv){
    if (argc != 4) {
        cerr << "Usage: Please add 1) Num of trials 2) Param for vertex 3) Planner" << endl;
        return 1;
    }

    // Record planning time for N trials
    int N = atoi(argv[1]);
    double n = atof(argv[2]);

    vector< vector<double> > time_stat;

    // Read and setup environment config
    string robot_config = "../config/robot_config_3d.csv",
            arena_config = "../config/arena_config_3d.csv",
            obs_config = "../config/obs_config_3d.csv";
    vector<SuperQuadrics> robot = generateSQ(robot_config, n),
                          arena = generateSQ(arena_config, n),
                          obs = generateSQ(obs_config, n);

    // Obstacle mesh
    vector<EMesh> obs_mesh;
    for(size_t i=0; i<obs.size(); i++) obs_mesh.push_back( getMesh(obs[i]) );

    // Boundary
    vector<double> b1 = {-arena[0].Shape.a[0],-arena[0].Shape.a[1],-arena[0].Shape.a[2]},
                   b2 = {arena[0].Shape.a[0],arena[0].Shape.a[1],arena[0].Shape.a[2]};

    // Start and goal setup
    inputFile file;
    string file_endpt = "../config/endPts_3d.csv";
    vector<vector<double> > endPts = file.parse2DCsvFile(file_endpt);

    std::ofstream outfile;
    outfile.open("time3D_ompl.csv", std::ios_base::app);
    outfile << "PLANNER,SUCCESS,TOTAL_TIME,ASTAR_TIME,CONST_TIME,RANDOM_CONF,GRAPH_NODES,GRAPH_EDGES,PATH_CONFIG,VALID_SPACE\n";
    outfile.close();
    // Planner number: PRM:0, PRMstar:1, RRT:2, RRTconnect:3
    int planner_used =atoi(argv[3]);
    for(int i=0; i<N; i++){
        FCLtester3D tester(b1, b2, robot, arena, obs, obs_mesh, planner_used);
        tester.plan(endPts[0], endPts[1], i);
    }

    return 0;
}
