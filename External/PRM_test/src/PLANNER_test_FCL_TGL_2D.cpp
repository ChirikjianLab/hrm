#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include "util/FileReader.h"
#include "geometry/SuperEllipse.h"
#include "tester/FCLTGLtester.h"

using namespace Eigen;
using namespace std;


int main(int argc, char ** argv){
   	if (argc <= 7) {
        cerr<< "Usage: Please add 1) configuration file 2)Agent 3)Arena 4)Obstacles 5)int indicating which planner to use 6) N iteratons" << endl;
        return 1;
    }

	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    
  	std::vector<SuperEllipse> robot_parts;
	std::vector<SuperEllipse> arena_parts;
	std::vector<SuperEllipse> obstacles;
 
    string file_endpt = argv[1];
    FileReader fr;
    vector<vector<double> > endPts = fr.parse2DCsvFile(file_endpt);

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
    vector<vector<double> > rob_config = fr.parse2DCsvFile(file_robConfig);
    for(int j=0; j<rob_config.size(); j++){
      SuperEllipse robot_aux = {{rob_config[j][0],rob_config[j][1],rob_config[j][2],
                                rob_config[j][3],rob_config[j][4],rob_config[j][5]}, b0};	      
      robot_parts.push_back(robot_aux);  
    }

    // Environment
    // Read environment config file
    string file_arenaConfig = argv[3];
    vector<vector<double> > arena_config = fr.parse2DCsvFile(file_arenaConfig);
    // Arena and Obstacles as class of SuperEllipse
    for(int j=0; j<arena_config.size(); j++){
      SuperEllipse arena_aux = {{arena_config[j][0],arena_config[j][1],arena_config[j][2],
                                arena_config[j][3],arena_config[j][4],arena_config[j][5]}, b0};	      
      arena_parts.push_back(arena_aux);  
    }

    string file_obsConfig = argv[4];
    vector<vector<double> > obs_config = fr.parse2DCsvFile(file_obsConfig);
    for(int j=0; j<obs_config.size(); j++){
      SuperEllipse obs_aux = {{obs_config[j][0],obs_config[j][1],obs_config[j][2],
                                obs_config[j][3],obs_config[j][4],obs_config[j][5]}, b0};	      
      obstacles.push_back(obs_aux);  
    }   

    //Reading meshes
    robot_parts[0].readingVerticesNTriangles(argv[7], argv[8]);  
    arena_parts[0].readingVerticesNTriangles(argv[9], argv[10]);  
    for(int i=0;i<obstacles.size();i++){
        int index = (i*2) +11;
        obstacles[i].readingVerticesNTriangles(argv[index], argv[index+1]);
        //obstacles[i].printTrianglesVertices();
    }

    

    std::ofstream outfile;
    outfile.open("times.csv", std::ios_base::app);
    outfile << "PLANNER,SUCCESS,TOTAL_TIME,ASTAR_TIME,CONST_TIME,RANDOM_CONF,GRAPH_NODES,GRAPH_EDGES,PATH_CONFIG,VALID_SPACE\n";
    outfile.close(); 
    //Planner number: PRM:0, PRMstar:1, RRT:2, RRTconnect:3
    int N = atoi(argv[6]);  
    int planner_used =atoi(argv[5]);
    for(int i = 0; i < N; i++){
        FCLTGLtester tester(b1,b2,arena_parts, robot_parts, obstacles, planner_used);
        tester.plan(start, goal, i);
    }
    

    return 0;
}
