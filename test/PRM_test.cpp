#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
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

#include <src/geometry/superellipse.h>

#include <unsupported/Eigen/Polynomials>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

#define pi 3.1415926
#define sgn(v) ( ( (v) < 0 ) ? -1 : ( (v) > 0 ) )


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
      ob::PlannerStatus solved = ss_->solve();

      planTime = ss_->getLastPlanComputationTime();
      flag = double(solved.operator bool());

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
          		bool aux = checkASC(robot_config, obstacles[i]);
          		if(aux == false){
            		res = false;
          		}
        	}
        	if(res == false){
          		return res;
        	}
            for(int k=0;k<arena.size();k++){
				res = checkASCArena(robot_config, arena[k]);
				if(res == false){
					return res;
				}
			}		
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
      Matrix2d A_i_ = robot_.rot2(robot_.a[2]);
      Matrix2d A_j_ = obs_.rot2(obs_.a[2]);
      bool res = robot_.algebraic_separation_condition(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
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
      coeff_canon_j_ << (arena_.a[0]-max), (arena_.a[1]-max);
      Vector2d r_i_;
      r_i_ << robot_.a[4], robot_.a[5];
      Vector2d r_j_;
      r_j_ << arena_.a[4], arena_.a[5];
      Matrix2d A_i_ = robot_.rot2(robot_.a[2]);
      Matrix2d A_j_ = arena_.rot2(arena_.a[2]);
      bool aux = robot_.algebraic_separation_condition(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
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
    double b1=-65.0,b2=65.0;

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
    int N = 50;
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

