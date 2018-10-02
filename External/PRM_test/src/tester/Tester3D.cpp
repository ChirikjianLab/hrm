#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/geometric/planners/prm/PRMstar.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <string>

#include <unsupported/Eigen/Polynomials>

#include "util/RotationUtil.h"
#include "tester/Tester3D.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

typedef Matrix<double,4,1> Vector4d;

Tester3D::Tester3D(double lowBound, double highBound, std::vector<SuperEllipse3D> arena_, std::vector<SuperEllipse3D> robot_, std::vector<SuperEllipse3D> obs_, int id){        
      
    arena = arena_;
    robot = robot_;
    obstacles = obs_;      
    id_planner = id;

    auto space(std::make_shared<ob::SE3StateSpace>());
    ob::RealVectorBounds bounds(3);
    bounds.setLow(lowBound);
    bounds.setHigh(highBound);
    space->setBounds(bounds);
    ss_ = std::make_shared<og::SimpleSetup>(space);   

    ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
    space->setup();
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(1/ space->getMaximumExtent());

    if(id_planner == 0){
        ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation())); 
    }
    if(id_planner == 1){
        ss_->setPlanner(std::make_shared<og::PRMstar>(ss_->getSpaceInformation())); 
    }
    if(id_planner == 2){
        ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation())); 
    }
    if(id_planner == 3){
        ss_->setPlanner(std::make_shared<og::RRTConnect>(ss_->getSpaceInformation())); 
    }

}

bool Tester3D::compareStates(std::vector<double> goal_config, std::vector<double> last_config){
    bool res = true;
    for(int i = 0; i < 7;i++){
        if(abs(last_config[i] - goal_config[i]) > 0.1){
            res = false;
        }
    }
    return res;
}

bool Tester3D::plan(std::vector<double> start_, std::vector<double> goal_, int n){
    if (!ss_){
      return false;
    }
    
    RotationUtil ru;

    ob::ScopedState<> start(ss_->getStateSpace());
    start->as<ob::SE3StateSpace::StateType>()->setXYZ(start_[0],start_[1],start_[2]);
    start->as<ob::SE3StateSpace::StateType>()->rotation().setAxisAngle(start_[3],start_[4],start_[5],start_[6]);
        
    ob::ScopedState<> goal(ss_->getStateSpace());
    goal->as<ob::SE3StateSpace::StateType>()->setXYZ(goal_[0],goal_[1],goal_[2]);
    goal->as<ob::SE3StateSpace::StateType>()->rotation().setAxisAngle(goal_[3],goal_[4],goal_[5],goal_[6]);
        
    ss_->setStartAndGoalStates(start, goal);
    ss_->setup();
    ss_->print();

    std::cout << "Planning..."<< std::endl;
    ob::PlannerStatus solved = ss_->solve(100);

    /*Getting times*/
    //flag = int(solved.operator bool());
    total_time = ss_->getLastPlanComputationTime();
    astar_time = ss_->getPlanner()->getAstarTime();
    construct_time = ss_->getPlanner()->getConstrucTime();
    total_random = ss_->getPlanner()->getTotalRandomConfig();

    //Getting graph info
    ob::PlannerData pd(ss_->getSpaceInformation());
    ss_->getPlannerData(pd);
    nodes_graph = pd.numVertices();
    edges_graph = pd.numEdges();
   
    // Store all the valid states
    ofstream file_state;   
    const ob::State *state;
    file_state.open("prm_state_"+std::to_string(n)+".csv");
    for(size_t i=0; i<pd.numVertices(); i++){
        state = pd.getVertex(i).getState()->as<ob::State>();
        Quaterniond aux_q = ru.toQuaternionType(state->as<ob::SE3StateSpace::StateType>()->rotation().x,state->as<ob::SE3StateSpace::StateType>()->rotation().y,state->as<ob::SE3StateSpace::StateType>()->rotation().z,state->as<ob::SE3StateSpace::StateType>()->rotation().w);
        Vector4d aux_axis = ru.getAngleAxisfromQuaternion(aux_q);
        file_state << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
                   << aux_axis(0,0) << ","
                   << aux_axis(1,0) << ","
                   << aux_axis(2,0) << ","
                   << aux_axis(3,0) << "\n";
    }
    file_state.close();
      
    // Store all the valid edges
    ofstream file_edge;
    file_edge.open("prm_edge_"+std::to_string(n)+".csv");
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
      ss_->getSolutionPath().print(std::cout);
      //Storing solution in a file 
      const std::vector<ob::State*> &states = ss_->getSolutionPath().getStates();
      ob::State *state;
      ofstream file_traj;
      file_traj.open("prm_path_"+ std::to_string(n) +".csv");
      for( size_t i = 0 ; i < states.size( ) ; ++i ){
        state = states[i]->as<ob::State >( );
        Quaterniond aux_q = ru.toQuaternionType(state->as<ob::SE3StateSpace::StateType>()->rotation().x,state->as<ob::SE3StateSpace::StateType>()->rotation().y,state->as<ob::SE3StateSpace::StateType>()->rotation().z,state->as<ob::SE3StateSpace::StateType>()->rotation().w);
        Vector4d aux_axis = ru.getAngleAxisfromQuaternion(aux_q);
        file_traj << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
                   << aux_axis(0,0) << ","
                   << aux_axis(1,0) << ","
                   << aux_axis(2,0) << ","
                   << aux_axis(3,0) << "\n";
      }
        
      file_traj.close();
      std::vector<double> final_conf;
      final_conf.resize(7);
      final_conf[0] = state->as<ob::SE3StateSpace::StateType>()->getX();
      final_conf[1] = state->as<ob::SE3StateSpace::StateType>()->getY();
      final_conf[2] = state->as<ob::SE3StateSpace::StateType>()->getZ();
      final_conf[3] = state->as<ob::SE3StateSpace::StateType>()->rotation().x;
      final_conf[4] = state->as<ob::SE3StateSpace::StateType>()->rotation().y;
      final_conf[5] = state->as<ob::SE3StateSpace::StateType>()->rotation().z;
      final_conf[6] = state->as<ob::SE3StateSpace::StateType>()->rotation().w;

      aux_flag = compareStates(goal_, final_conf);    

      // Smooth path
      ss_->getSolutionPath().interpolate(50);
        
      // print the path to screen
      //ss_->getSolutionPath().printAsMatrix(std::cout);
      //Storing solution in a file
      const std::vector<ob::State*> &s_states = ss_->getSolutionPath().getStates();

      ofstream file_smooth_traj;
      file_smooth_traj.open("prm_smooth_path_"+std::to_string(n)+".csv");
      for( size_t i = 0 ; i < s_states.size( ) ; ++i ){
        state = s_states[i]->as<ob::State >( );
        Quaterniond aux_q = ru.toQuaternionType(state->as<ob::SE3StateSpace::StateType>()->rotation().x,state->as<ob::SE3StateSpace::StateType>()->rotation().y,state->as<ob::SE3StateSpace::StateType>()->rotation().z,state->as<ob::SE3StateSpace::StateType>()->rotation().w);
        Vector4d aux_axis = ru.getAngleAxisfromQuaternion(aux_q);
        file_smooth_traj << state->as<ob::SE3StateSpace::StateType>()->getX() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->getY() << ","
                   << state->as<ob::SE3StateSpace::StateType>()->getZ() << ","
                   << aux_axis(0,0) << ","
                   << aux_axis(1,0) << ","
                   << aux_axis(2,0) << ","
                   << aux_axis(3,0) << "\n";
      }
      file_smooth_traj.close();
      flag = int(aux_flag); 

      //Saving times and configs in file
      std::ofstream outfile;
      outfile.open("times.csv", std::ios_base::app);
      outfile << id_planner<<","<<flag <<","<< total_time<<"," << astar_time <<","<< construct_time<<","<<
       total_random<<","<<nodes_graph<<"," <<edges_graph<<"," <<nodes_path<<"," <<valid_space<<"\n";
      outfile.close();        
      return true;
    } 
    return false;
  }

bool Tester3D::isStateValid(const ob::State *state) const{
    bool res  = true;        
    double x = state->as<ob::SE3StateSpace::StateType>()->getX();
    double y = state->as<ob::SE3StateSpace::StateType>()->getY();
    double z = state->as<ob::SE3StateSpace::StateType>()->getY();
    double qx = state->as<ob::SE3StateSpace::StateType>()->rotation().x;
    double qy = state->as<ob::SE3StateSpace::StateType>()->rotation().y;
    double qz = state->as<ob::SE3StateSpace::StateType>()->rotation().z;
    double qw = state->as<ob::SE3StateSpace::StateType>()->rotation().w;

    RotationUtil ru;

    Quaterniond aux_q = ru.toQuaternionType(qx, qy, qz, qw);
    //cout<<"qx: "<< aux_q.x()<<" qy: "<<aux_q.y()<<" qz: "<<aux_q.z()<< " qw: "<<aux_q.w()<<endl;
    Vector4d aux_axis = ru.getAngleAxisfromQuaternion(aux_q);
    //cout<<"ax: "<<aux_axis(0,0)<<" ay: "<<aux_axis(1,0)<<" az: "<<aux_axis(2,0)<<" as: " << aux_axis(3,0)<<endl;

    for(unsigned int j=0; j<robot.size();j++){
		SuperEllipse3D robot_config = {{robot[j].a[0],robot[j].a[1],robot[j].a[2],aux_axis(0,0),aux_axis(1,0),aux_axis(2,0),aux_axis(3,0),robot[j].a[7],x,y,z}, robot[j].num};
        robot_config.initQuaternion();
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
		/*for(int k=0;k<arena.size();k++){
			res = checkSeparationArena(robot_config, arena[k]);
			if(res == false){
				return res;
			}
		}*/		
	}
    return res; 
}

  
