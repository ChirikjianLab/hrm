#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <fcl/fcl.h>
#include "fcl/geometry/shape/ellipsoid.h"
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>

#include <unsupported/Eigen/Polynomials>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

#define pi 3.1415926
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )

typedef Matrix<double,4,1> Vector4d;
using GeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;

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
      //Fix this function later
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

    bool fcl_separation(Vector2d coeff_canon_i_, Vector2d coeff_canon_j_, Vector2d r_i_, Vector2d r_j_, double theta_i_, double theta_j_){
      bool res = false;
      GeometryPtr_t i_geometry(new fcl::Ellipsoid<double>(coeff_canon_i_(0,0), coeff_canon_i_(1,0), 0.0));
      fcl::Matrix3<double> rotation_i_(fcl::AngleAxis<double>(theta_i_, fcl::Vector3<double>::UnitZ()));
      fcl::Vector3<double> T_i_(r_i_(0,0), r_i_(1,0), 0.0);
      fcl::Transform3<double> i_transform = fcl::Transform3<double>::Identity();
      i_transform.translation() = (T_i_);
      i_transform.linear() = rotation_i_;
      fcl::CollisionObject<double> i_ellipsoid(i_geometry, i_transform);

      GeometryPtr_t j_geometry(new fcl::Ellipsoid<double>(coeff_canon_j_(0,0), coeff_canon_j_(1,0), 0.0));
      fcl::Matrix3<double> rotation_j_(fcl::AngleAxis<double>(theta_j_, fcl::Vector3<double>::UnitZ()));
      fcl::Vector3<double> T_j_(r_j_(0,0), r_j_(1,0), 0.0);
      fcl::Transform3<double> j_transform = fcl::Transform3<double>::Identity();
      j_transform.translation() = (T_j_);
      j_transform.linear() = rotation_j_;
      fcl::CollisionObject<double> j_ellipsoid(j_geometry, j_transform);

      fcl::CollisionRequest<double> request;
      fcl::CollisionResult<double> result;
      
      fcl::collide(&i_ellipsoid, &j_ellipsoid, request, result);
      if (result.numContacts() > 0){
        //std::cout << "In collision" << std::endl;
        res = false;
      }else{
        //std::cout << "Not in collision" << std::endl;
        res = true;
      }

      return res;
    }

};


class PRMtester{
  public:
    PRMtester(double lowBound, double highBound, std::vector<SuperEllipse> arena_, std::vector<SuperEllipse> robot_, std::vector<SuperEllipse> obs_){        
      arena = arena_;
      robot = robot_;
      obstacles = obs_;      

      auto space(std::make_shared<ob::SE2StateSpace>());
      ob::RealVectorBounds bounds(2);
      bounds.setLow(lowBound);
      bounds.setHigh(highBound);
      space->setBounds(bounds);
      ss_ = std::make_shared<og::SimpleSetup>(space);   

      ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
      space->setup();
      ss_->getSpaceInformation()->setStateValidityCheckingResolution(1/ space->getMaximumExtent());
      ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));       
    }

    bool plan(std::vector<double> start_, std::vector<double> goal_){
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
      ss_->print();

      std::cout << "Planning..."<< std::endl;
      ob::PlannerStatus solved = ss_->solve(20.0);

      planTime = ss_->getLastPlanComputationTime();
      flag = double(solved.operator bool());

      ob::PlannerData pd(ss_->getSpaceInformation());
      ss_->getPlannerData(pd);

      // Store all the valid states
      ofstream file_state;
      cout << pd.numVertices() << ' ' << pd.numEdges() << endl;
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
          for(int j=0; j<edge[i].size(); j++) file_edge << int(i) << ' ' << int(edge[i][j]) << '\n';
      }

      if(solved){
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss_->getSolutionPath().print(std::cout);
        //Storing solution in a file 
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

        // Smooth path
        ss_->getSolutionPath().interpolate(50);
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss_->getSolutionPath().printAsMatrix(std::cout);
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

        return true;
      }
      return false;
    }

    og::SimpleSetupPtr ss_;
    double planTime,flag;
    int N_v_sample, N_v, N_p;

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
          		bool aux = checkSeparation(robot_config, obstacles[i]);
          		if(aux == false){
            		res = false;
          		}
        	}
        	if(res == false){
          		return res;
        	}
//			for(int k=0;k<arena.size();k++){
//				res = checkSeparationArena(robot_config, arena[k]);
//				if(res == false){
//					return res;
//				}
//			}
		}
        return res; 
    }

    //Returns true when separated and false when overlapping 
    bool checkSeparation(SuperEllipse robot_, SuperEllipse obs_) const{      
      Vector2d coeff_canon_i_;
      coeff_canon_i_ << robot_.a[0], robot_.a[1];
      Vector2d coeff_canon_j_;
      coeff_canon_j_ << obs_.a[0], obs_.a[1];
      Vector2d r_i_;
      r_i_ << robot_.a[4], robot_.a[5];
      Vector2d r_j_;
      r_j_ << obs_.a[4], obs_.a[5];
      bool res = robot_.fcl_separation(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, robot_.a[2], obs_.a[2]);
      /*if(res){
        std::cout<<"True"<<std::endl;
      }else{
        std::cout<<"False"<<std::endl;
      }*/
      return res;       
    }

    //Returns true when separated and false when overlapping
    bool checkSeparationArena(SuperEllipse robot_, SuperEllipse arena_) const{
      bool res = true; 
      double max = 0;
      for(int i=0;i<3;i++){
        if(robot_.a[i]>max){
          max = robot_.a[i];
        }
      }     
      Vector2d coeff_canon_i_;
      coeff_canon_i_ << robot_.a[0], robot_.a[1];
      Vector2d coeff_canon_j_;
      coeff_canon_j_ << (arena_.a[0]-max), (arena_.a[1]-max);
      Vector2d r_i_;
      r_i_ << robot_.a[4], robot_.a[5];
      Vector2d r_j_;
      r_j_ << arena_.a[4], arena_.a[5];
      bool aux = robot_.fcl_separation(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, robot_.a[2], arena_.a[2]);
      if(aux == true){
        res = false;
      }       
      return res;
    }

    std::vector<SuperEllipse> arena;
    std::vector<SuperEllipse> robot;
    std::vector<SuperEllipse> obstacles;
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

int main(){
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // Number of points on the boundary
    int num = 50;

    // Robot
    // Read robot config file
    string file_robConfig = "../config/robotConfig.csv";
    vector<vector<double> > rob_config = parse2DCsvFile(file_robConfig);

    // Robot as a class of SuperEllipse
    vector<SuperEllipse> robot(rob_config.size());
    for(int j=0; j<rob_config.size(); j++){
        for(int i=0; i<6; i++) robot[j].a[i] = rob_config[0][i];
        robot[j].num = num;
    }

    // Environment
    // Read environment config file
    string file_arenaConfig = "../config/arenaConfig.csv";
    vector<vector<double> > arena_config = parse2DCsvFile(file_arenaConfig);

    string file_obsConfig = "../config/obsConfig.csv";
    vector<vector<double> > obs_config = parse2DCsvFile(file_obsConfig);

    string file_endpt = "../config/endPts.csv";
    vector<vector<double> > endPts = parse2DCsvFile(file_endpt);

    // Arena and Obstacles as class of SuperEllipse
    vector<SuperEllipse> arena(arena_config.size()), obs(obs_config.size());
    for(int j=0; j<arena_config.size(); j++){
        for(int i=0; i<6; i++) arena[j].a[i] = arena_config[j][i];
        arena[j].num = num;
    }
    for(int j=0; j<obs_config.size(); j++){
        for(int i=0; i<6; i++) obs[j].a[i] = obs_config[j][i];
        obs[j].num = num;
    }

    //Getting bounderies
    double b1=-45.0,b2=45.0;

    //Getting start configuration
    std::vector<double> start;
    start.resize(3);
    start[0] = endPts[0][0];
    start[1] = endPts[0][1];
    start[2] = endPts[0][2];

    //Getting goal configuration
    std::vector<double> goal;
    goal.resize(3);
    goal[0] = endPts[1][0];
    goal[1] = endPts[1][1];
    goal[2] = endPts[1][2];


    // Record planning time for N trials
    int N = 1;
    vector<double> time_stat[N];

    for(int i=0; i<N; i++){
        // Main algorithm
        PRMtester tester(b1,b2,arena, robot, obs);
        tester.plan(start, goal);

        // Planning Time and Path Cost
        cout << "Total Planning Time: " << tester.planTime << 's' << endl;

        time_stat[i].push_back(tester.planTime);
        time_stat[i].push_back(tester.flag);
    }

    ofstream file_time;
    file_time.open("planTime_prm.csv");
    for(int i=0; i<N; i++) file_time << time_stat[i][0] << ' ' << time_stat[i][1] << "\n";
    file_time.close();

    return 0;
}
