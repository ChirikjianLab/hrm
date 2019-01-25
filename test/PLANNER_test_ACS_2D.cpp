#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/geometric/planners/prm/PRMstar.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <string>

#include <unsupported/Eigen/Polynomials>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

#define pi 3.1415926
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )

class SuperEllipse{
public:
    /* Parameters of superellipse
        a[0], a[1]: semi-axes length;
        a[2]      : rotational angle;
        a[3]      : epsilon;
        a[4], a[5]: position of the center.
    */
    double a[6];
    // Number of points on boundary
    int num;

    // Functions
    //SuperEllipse(double a[6], int num);
    MatrixXd originShape(double a[6], int num){
        double th;
        Vector2d x;
        MatrixXd trans(2,num), X(2,num);

        for(int i=0; i<num; i++){
            th = 2*i*pi/(num-1);

            x(0,0) = a[0] * expFun(th,a[3],0);
            x(1,0) = a[1] * expFun(th,a[3],1);

            trans(0,i) = a[4]; trans(1,i) = a[5];
            X(0,i) = x(0,0); X(1,i) = x(1,0);
        }
        X = Rotation2Dd(a[2]).matrix() * X + trans;
        return X;
    }
    
    double expFun(double th, double p, bool func){
        return (func == 0) ? sgn(cos(th)) * pow(abs(cos(th)), p) : sgn(sin(th)) * pow(abs(sin(th)), p) ;
    }

    /*
    * coeff_canon_i/j_ semi axis, r_i/j_ centers, A_i/j_ rotation matrix
    * If separated return False, if in collision returns True
    */
    bool algebraic_condition_separation(Vector2d coeff_canon_i_, Vector2d coeff_canon_j_, Vector2d r_i_, Vector2d r_j_, Matrix2d A_i_, Matrix2d A_j_){
        //Surface i
        Matrix3d A;
        A << 1/pow(coeff_canon_i_(0,0),2), 0 ,0,
                0, 1/pow(coeff_canon_i_(1,0),2), 0,
                0, 0, -1;
        //Surface j
        Matrix3d B;
        B << 1/pow(coeff_canon_j_(0,0),2), 0 ,0,
                0, 1/pow(coeff_canon_j_(1,0),2), 0,
                0, 0, -1;
        
        //Rigid body transformations
        Matrix3d T_i;
        T_i << A_i_(0, 0), A_i_(0, 1), r_i_(0,0),
                A_i_(1, 0), A_i_(1, 1), r_i_(1,0),
                0,0,1;

        Matrix3d T_j;
        T_j << A_j_(0, 0), A_j_(0, 1), r_j_(0,0),
                A_j_(1, 0), A_j_(1, 1), r_j_(1,0),
                0,0,1;
        Matrix3d Ma = T_i;
        Matrix3d Mb = T_j;

        //aij belongs to A in det(lambda*A - Ma'*(Mb^-1)'*B*(Mb^-1)*Ma)
        Matrix3d a = A;
        //bij belongs to b = Ma'*(Mb^-1)'*B*(Mb^-1)*Ma matmul
        Matrix3d aux = Mb.inverse()* Ma;
        Matrix3d b = aux.transpose() * B * aux;

        //Coefficients of the Characteristic Polynomial
        double T4 = -a(0,0) * a(1,1) * a(2,2);
        double T3 = (a(0,0)*a(1,1)*b(2,2)) + (a(0,0)*a(2,2)*b(1,1)) + (a(1,1)*a(2,2)*b(0,0));
        double T2 = (a(0,0)*b(1,2)*b(2,1)) - (a(0,0)*b(1,1)*b(2,2)) - (a(1,1)*b(0,0)*b(2,2)) + (a(1,1)*b(0,2)*b(2,0)) - (a(2,2)*b(0,0)*b(1,1)) + (a(2,2)*b(0,1)*b(1,0));
        double T1 = (b(0,0)*b(1,1)*b(2,2)) - (b(0,0)*b(1,2)*b(2,1)) - (b(0,1)*b(1,0)*b(2,2)) + (b(0,1)*b(1,2)*b(2,0)) + (b(0,2)*b(1,0)*b(2,1)) - (b(0,2)*b(1,1)*b(2,0));
        double T0 = 0;

        typedef Matrix<double,5,1> Vector5d;
        
        //Solving characteristic polynomial
        Vector5d characteristic_polynomial;
        characteristic_polynomial << T0,T1,T2,T3,T4;

        //cout << "characteristic_polynomial: " << characteristic_polynomial.transpose() << endl;
        PolynomialSolver<double, 4>psolve;
        psolve.compute(characteristic_polynomial);
        //cout << "Complex roots: " << psolve.roots().transpose() << endl;
        
        /*Checking roots conditions
        * Two different negative real roots : separated
        * If real part is =, then they touch in one point
        * If there are more or less than 2 negative real roots, then they touch.
        */
        
        Eigen::MatrixXd roots = Eigen::MatrixXd::Zero(6,1);
        int j = 0;
        for(int i=0;i<psolve.roots().size();i++){
            if(psolve.roots()[i].real() < -0.000001){
                //cout<<psolve.roots()[i].real()<<endl;
                roots(j,0) = psolve.roots()[i].real();
                j++;
            }
        }
        if(j == 2){
            if(roots(0,0) != roots(1,0)){
                return true;
            }
            if(abs(roots(0,0) - roots(1,0))<0.001){
                return false;
            }
        }
        return false;
    }

    Matrix2d rotation_angle_axis(double theta){
        Matrix2d S;
        S << 0, -1, 1,0;
        Matrix2d R;
        MatrixXd id = MatrixXd::Identity(2,2);
        R = id + ((sin(theta)) * S) + ((1 - cos(theta))*(S * S));
        return R;
    }
};


class PRMtester{
public:
    PRMtester(double xBound, double yBound, std::vector<SuperEllipse> arena_, std::vector<SuperEllipse> robot_, std::vector<SuperEllipse> obs_, int id){
        arena = arena_;
        robot = robot_;
        obstacles = obs_;
        id_planner = id;

        auto space(std::make_shared<ob::SE2StateSpace>());
        ob::RealVectorBounds bounds(2);
        bounds.setHigh(0,xBound);
        bounds.setLow(0,-xBound);
        bounds.setHigh(1,yBound);
        bounds.setLow(1,-yBound);
        space->setBounds(bounds);
        ss_ = std::make_shared<og::SimpleSetup>(space);

        ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
        space->setup();
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(1/ space->getMaximumExtent());

        if(id_planner == 1){
            ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
        }
        if(id_planner == 2){
            ss_->setPlanner(std::make_shared<og::PRMstar>(ss_->getSpaceInformation()));
        }
        if(id_planner == 3){
            ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
        }
        if(id_planner == 4){
            ss_->setPlanner(std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
        }

    }

    bool compareStates(std::vector<double> goal_config, std::vector<double> last_config){
        bool res = false;
        if((abs(last_config[0] - goal_config[0]) < 0.1) && (abs(last_config[1] - goal_config[1]) < 0.1) && (abs(last_config[2] - goal_config[2]) < 0.1) ){
            res = true;
        }
        return res;

    }
    bool plan(std::vector<double> start_, std::vector<double> goal_, int n){
        if (!ss_){
            return false;
        }
        ob::ScopedState<> start(ss_->getStateSpace());
        start->as<ob::SE2StateSpace::StateType>()->setX(start_[0]);
        start->as<ob::SE2StateSpace::StateType>()->setY(start_[1]);
        start->as<ob::SE2StateSpace::StateType>()->setYaw(start_[2]);
        
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal->as<ob::SE2StateSpace::StateType>()->setX(goal_[0]);
        goal->as<ob::SE2StateSpace::StateType>()->setY(goal_[1]);
        goal->as<ob::SE2StateSpace::StateType>()->setYaw(goal_[2]);
        
        ss_->setStartAndGoalStates(start, goal);
        ss_->setup();
        // ss_->print();

        std::cout << "Planning..."<< std::endl;
        ob::PlannerStatus solved = ss_->solve(200);

        /*Getting times*/
        //flag = int(solved.operator bool());
        total_time = ss_->getLastPlanComputationTime();
        //        astar_time = ss_->getPlanner()->getAstarTime();
        //        construct_time = ss_->getPlanner()->getConstrucTime();
        //        total_random = ss_->getPlanner()->getTotalRandomConfig();

        //Getting graph info
        ob::PlannerData pd(ss_->getSpaceInformation());
        ss_->getPlannerData(pd);
        nodes_graph = pd.numVertices();
        edges_graph = pd.numEdges();

        // Store all the valid states
        ofstream file_state;
        const ob::State *state;
        file_state.open("prm_state.csv");
        for(size_t i=0; i<pd.numVertices(); i++){
            state = pd.getVertex(i).getState()->as<ob::State>();
            file_state << state->as<ob::SE2StateSpace::StateType>()->getX() << ","
                       << state->as<ob::SE2StateSpace::StateType>()->getY() << ","
                       << state->as<ob::SE2StateSpace::StateType>()->getYaw() << "\n";
        }
        file_state.close();

        // Store all the valid edges
        ofstream file_edge;
        file_edge.open("prm_edge.csv");
        vector< vector<unsigned int> > edge(pd.numVertices());
        for(size_t i=0; i<pd.numVertices(); i++){
            pd.getEdges(i,edge[i]);
            for(int j=0; j<edge[i].size(); j++) file_edge << int(i) << " " << int(edge[i][j]) << "\n";
        }
        file_edge.close();

        nodes_path = ss_->getSolutionPath().getStates().size();
        valid_space = ss_->getSpaceInformation()->probabilityOfValidState(1000);

        bool aux_flag = false;
        if(solved){
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            // ss_->getSolutionPath().print(std::cout);
            // Storing solution in a file
            const std::vector<ob::State*> &states = ss_->getSolutionPath().getStates();
            ob::State *state;
            ofstream file_traj;
            file_traj.open("prm_path.csv");
            for( size_t i = 0 ; i < states.size( ) ; ++i ){
                state = states[i]->as<ob::State >( );
                file_traj << state->as<ob::SE2StateSpace::StateType>()->getX() << ","
                          << state->as<ob::SE2StateSpace::StateType>()->getY() << ","
                          << state->as<ob::SE2StateSpace::StateType>()->getYaw() << "\n";
            }

            file_traj.close();
            std::vector<double> final_conf;
            final_conf.resize(3);
            final_conf[0] = state->as<ob::SE2StateSpace::StateType>()->getX();
            final_conf[1] = state->as<ob::SE2StateSpace::StateType>()->getY();
            final_conf[2] = state->as<ob::SE2StateSpace::StateType>()->getYaw();
            aux_flag = compareStates(goal_, final_conf);

            // Smooth path
            ss_->getSolutionPath().interpolate(50);

            // print the path to screen
            //ss_->getSolutionPath().printAsMatrix(std::cout);
            //Storing solution in a file
            const std::vector<ob::State*> &s_states = ss_->getSolutionPath().getStates();

            ofstream file_smooth_traj;
            file_smooth_traj.open("prm_smooth_path.csv");
            for( size_t i = 0 ; i < s_states.size( ) ; ++i ){
                state = s_states[i]->as<ob::State >( );
                file_smooth_traj << state->as<ob::SE2StateSpace::StateType>()->getX() << ","
                                 << state->as<ob::SE2StateSpace::StateType>()->getY() << ","
                                 << state->as<ob::SE2StateSpace::StateType>()->getYaw() << "\n";
            }
            file_smooth_traj.close();
            flag = int(aux_flag);

            // Saving times and configs in file
            std::ofstream outfile;
            outfile.open("time_prm.csv", std::ios_base::app);
            //            outfile << id_planner<<","<<flag <<","<< total_time<<"," << astar_time <<","<< construct_time<<","<<
            //                       total_random<<","<<nodes_graph<<","<<edges_graph<<","<<nodes_path<<","<<valid_space<<"\n";

            outfile << id_planner <<","<< flag <<","<< total_time <<"," <<
                       nodes_graph <<","<< edges_graph <<","<< nodes_path <<","<< valid_space<<"\n";
            outfile.close();
            return true;
        }
        return false;
    }

private:

    bool isStateValid(const ob::State *state) const{
        bool res  = true;
        double x = state->as<ob::SE2StateSpace::StateType>()->getX();
        double y = state->as<ob::SE2StateSpace::StateType>()->getY();
        double yaw = state->as<ob::SE2StateSpace::StateType>()->getYaw();

        for(unsigned int j=0; j<robot.size();j++){
            SuperEllipse robot_config = {{robot[j].a[0],robot[j].a[1],yaw,robot[j].a[3],x,y}, robot[j].num};
            //Checking collision against obstacles
            for(unsigned int i=0; i<obstacles.size(); i++){
                bool aux = checkASC(robot_config, obstacles[i]);
                if(aux == false){
                    res = false;
                }
            }
            if(res == false){
                return res;
            }
            //			for(int k=0;k<arena.size();k++){
            //				res = checkASCArena(robot_config, arena[k]);
            //				if(res == false){
            //					return res;
            //				}
            //			}
        }
        return res;
    }

    //Returns true when separated and false when overlapping
    bool checkASC(SuperEllipse robot_, SuperEllipse obs_) const{
        Vector2d coeff_canon_i_;
        coeff_canon_i_ << robot_.a[0], robot_.a[1];
        Vector2d coeff_canon_j_;
        coeff_canon_j_ << obs_.a[0], obs_.a[1];
        Vector2d r_i_;
        r_i_ << robot_.a[4], robot_.a[5];
        Vector2d r_j_;
        r_j_ << obs_.a[4], obs_.a[5];
        Matrix2d A_i_ = robot_.rotation_angle_axis(robot_.a[2]);
        Matrix2d A_j_ = obs_.rotation_angle_axis(obs_.a[2]);
        bool res = robot_.algebraic_condition_separation(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
        /*if(res){
        std::cout<<"True"<<std::endl;
      }else{
        std::cout<<"False"<<std::endl;
      }*/
        return res;
    }

    //Returns true when separated and false when overlapping
    bool checkASCArena(SuperEllipse robot_, SuperEllipse arena_) const{
        bool res = true;
        double max = 0;
        for(int i=0;i<2;i++){
            if(robot_.a[i]>max){
                max = robot_.a[i];
            }
        }
        Vector2d coeff_canon_i_;
        coeff_canon_i_ << robot_.a[0], robot_.a[1];
        Vector2d coeff_canon_j_;
        //coeff_canon_j_ << (arena_.a[0]-(max*2)), (arena_.a[1]-(max*2));
        coeff_canon_j_ << (arena_.a[0]-(max)), (arena_.a[1]-(max));
        Vector2d r_i_;
        r_i_ << robot_.a[4], robot_.a[5];
        Vector2d r_j_;
        r_j_ << arena_.a[4], arena_.a[5];


        Matrix2d A_i_ = robot_.rotation_angle_axis(robot_.a[2]);
        Matrix2d A_j_ = arena_.rotation_angle_axis(arena_.a[2]);
        bool aux = robot_.algebraic_condition_separation(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
        if(aux == true){
            res = false;
        }

        //If arena is a superellipsoid with epsilon = 0.001
        /*if((coeff_canon_i_(0,0)<-coeff_canon_j_(0,0) || coeff_canon_i_(0,0)>coeff_canon_j_(0,0)) &&
         (coeff_canon_i_(1,0)<-coeff_canon_j_(1,0) || coeff_canon_i_(1,0)>coeff_canon_j_(1,0)) ){
        res = false;
      } */
        return res;
    }

    og::SimpleSetupPtr ss_;
    std::vector<SuperEllipse> arena;
    std::vector<SuperEllipse> robot;
    std::vector<SuperEllipse> obstacles;

    unsigned int nodes_graph;
    unsigned int edges_graph;
    //    int total_random;
    int nodes_path;
    int flag;
    //    double astar_time;
    //    double construct_time;
    double total_time;
    double valid_space;
    double id_planner;
};

vector<vector<double>> parse2DCsvFile(string inputFileName) {
    vector<vector<double> > data;
    ifstream inputFile(inputFileName);
    int l = 0;

    while (inputFile) {
        l++;
        string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
            istringstream ss(s);
            vector<double> record;

            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(stof(line));
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << inputFileName << " line " << l
                         << endl;
                    e.what();
                }
            }

            data.push_back(record);
        }
    }

    if (!inputFile.eof()) {
        cerr << "Could not read file " << inputFileName << "\n";
        __throw_invalid_argument("File not found.");
    }

    return data;
}


int main(int argc, char ** argv){
    cout << argc << endl;
    if (argc != 7) {
        cerr<< "Usage: Please add 1) configuration file 2)Agent 3)Arena 4)Obstacles 5)int indicating which planner to use 6) N iteratons" << endl;
        return 1;
    }

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    
    std::vector<SuperEllipse> robot_parts;
    std::vector<SuperEllipse> arena_parts;
    std::vector<SuperEllipse> obstacles;

    string file_endpt = argv[1];
    vector<vector<double> > endPts = parse2DCsvFile(file_endpt);

    //Getting points in the boundaries of shapes
    int b0 = int(endPts[0][0]);
    
    //Getting bounderies
    double b1 = int(endPts[0][1]);
    double b2 = int(endPts[0][2]);
    
    //Getting start configuration
    std::vector<double> start;
    start.resize(3);
    start[0] = endPts[1][0];
    start[1] = endPts[1][1];
    start[2] = endPts[1][2];

    //Getting goal configuration
    std::vector<double> goal;
    goal.resize(3);
    goal[0] = endPts[2][0];
    goal[1] = endPts[2][1];
    goal[2] = endPts[2][2];

    // Robot
    // Read robot config file
    string file_robConfig = argv[2];
    vector<vector<double> > rob_config = parse2DCsvFile(file_robConfig);
    for(int j=0; j<rob_config.size(); j++){
        SuperEllipse robot_aux = {{rob_config[j][0],rob_config[j][1],rob_config[j][2],
                                   rob_config[j][3],rob_config[j][4],rob_config[j][5]}, b0};
        robot_parts.push_back(robot_aux);
    }

    // Environment
    // Read environment config file
    string file_arenaConfig = argv[3];
    vector<vector<double> > arena_config = parse2DCsvFile(file_arenaConfig);
    // Arena and Obstacles as class of SuperEllipse
    for(int j=0; j<arena_config.size(); j++){
        SuperEllipse arena_aux = {{arena_config[j][0],arena_config[j][1],arena_config[j][2],
                                   arena_config[j][3],arena_config[j][4],arena_config[j][5]}, b0};
        arena_parts.push_back(arena_aux);
    }

    string file_obsConfig = argv[4];
    vector<vector<double> > obs_config = parse2DCsvFile(file_obsConfig);
    for(int j=0; j<obs_config.size(); j++){
        SuperEllipse obs_aux = {{obs_config[j][0],obs_config[j][1],obs_config[j][2],
                                 obs_config[j][3],obs_config[j][4],obs_config[j][5]}, b0};
        obstacles.push_back(obs_aux);
    }

    std::ofstream outfile;
    outfile.open("time_prm.csv", std::ios_base::app);
    //    outfile << "PLANNER,SUCCESS,TOTAL_TIME,ASTAR_TIME,CONST_TIME,RANDOM_CONF,GRAPH_NODES,GRAPH_EDGES,PATH_CONFIG,VALID_SPACE\n";
    outfile << "PLANNER,SUCCESS,TOTAL_TIME,GRAPH_NODES,GRAPH_EDGES,PATH_CONFIG,VALID_SPACE\n";
    outfile.close();
    //Planner number: PRM:1, PRMstar:2, RRT:3, RRTconnect:4
    int N = atoi(argv[6]);
    int planner_used =atoi(argv[5]);
    for(int i = 0; i < N; i++){
        cout << i << endl;
        PRMtester tester(b1,b2,arena_parts, robot_parts, obstacles, planner_used);
        tester.plan(start, goal, i);
    }
    

    return 0;
}
