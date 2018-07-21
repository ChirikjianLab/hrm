#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/math/bv/OBBRSS.h>
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
    std::vector<fcl::Vector3<double>> vertices;
    std::vector<fcl::Triangle> triangles;
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

    bool readingVerticesNTriangles(std::string file_vertices, std::string file_triangles){        
        //Getting the vertices and triangles from files
        std::fstream inputvertices(file_vertices,std::ios_base::in);
	    if(!inputvertices){
            std::cout << "Error opening vertices file" << std::endl;
            return(1);
        }    
        double v1, v2;
        while(inputvertices >> v1 >> v2){
            fcl::Vector3<double> aux_vec;
            aux_vec << v1,v2,0.0;
            vertices.push_back(aux_vec);
        }
        inputvertices.close();

        std::fstream inputtriangles(file_triangles,std::ios_base::in);
	    if(!inputtriangles){
            std::cout << "Error opening triangle file" << std::endl;
            return(1);
        }    
        int t1, t2, t3;
        while(inputtriangles >> t1 >> t2 >> t3){
            fcl::Triangle aux_tri(t1,t2,t3);
            triangles.push_back(aux_tri);
        }
        inputtriangles.close();
        return true;
    }

    bool fcl_separation(SuperEllipse se_i_, SuperEllipse se_j_, Vector2d r_i_, Vector2d r_j_, double theta_i_, double theta_j_){
      bool res = false;
      fcl::BVHModel<fcl::OBBRSS<double>>* model_i_ = new fcl::BVHModel<fcl::OBBRSS<double>>();
      // add the mesh data into the BVHModel structure
      model_i_->beginModel();
      model_i_->addSubModel(se_i_.vertices, se_i_.triangles);
      model_i_->endModel();

      fcl::Matrix3<double> rotation_i_(fcl::AngleAxis<double>(theta_i_, fcl::Vector3<double>::UnitZ()));
      fcl::Vector3<double> T_i_(r_i_(0,0), r_i_(1,0), 0.0);
      fcl::Transform3<double> i_transform = fcl::Transform3<double>::Identity();
      i_transform.translation() = (T_i_);
      i_transform.linear() = rotation_i_;
      fcl::CollisionObject<double> i_superellipsoid(GeometryPtr_t(model_i_), i_transform);

      
      fcl::BVHModel<fcl::OBBRSS<double>>* model_j_ = new fcl::BVHModel<fcl::OBBRSS<double>>();
      model_j_->beginModel();
      model_j_->addSubModel(se_j_.vertices, se_j_.triangles);
      model_j_->endModel();
      fcl::Matrix3<double> rotation_j_(fcl::AngleAxis<double>(theta_j_, fcl::Vector3<double>::UnitZ()));
      fcl::Vector3<double> T_j_(r_j_(0,0), r_j_(1,0), 0.0);
      fcl::Transform3<double> j_transform = fcl::Transform3<double>::Identity();
      j_transform.translation() = (T_j_);
      j_transform.linear() = rotation_j_;
      fcl::CollisionObject<double> j_superellipsoid(GeometryPtr_t(model_j_), j_transform);

      fcl::CollisionRequest<double> request;
      fcl::CollisionResult<double> result;
      
      fcl::collide(&i_superellipsoid, &j_superellipsoid, request, result);
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
      ob::PlannerStatus solved = ss_->solve();

      if(solved){
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss_->getSolutionPath().print(std::cout);
        //Storing solution in a file 
        const std::vector<ob::State*> &states = ss_->getSolutionPath().getStates();
        ob::State *state;
        ofstream file_traj;
        file_traj.open("trajectory.csv");
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

private:

    bool isStateValid(const ob::State *state) const{
        bool res  = true;        
        double x = state->as<ob::SE2StateSpace::StateType>()->getX();
        double y = state->as<ob::SE2StateSpace::StateType>()->getY();
        double yaw = state->as<ob::SE2StateSpace::StateType>()->getYaw();

		for(unsigned int j=0; j<robot.size();j++){
			SuperEllipse robot_config = {{robot[j].a[0],robot[j].a[1],yaw,robot[j].a[3],x,y}, robot[j].num};
            robot_config.triangles = robot[j].triangles;
            robot_config.vertices = robot[j].vertices;
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
			for(int k=0;k<arena.size();k++){
				res = checkSeparationArena(robot_config, arena[k]);
				if(res == false){
					return res;
				}
			}		
		}
        return res; 
    }

    //Returns true when separated and false when overlapping 
    bool checkSeparation(SuperEllipse robot_, SuperEllipse obs_) const{      
      Vector2d r_i_;
      r_i_ << robot_.a[4], robot_.a[5];
      Vector2d r_j_;
      r_j_ << obs_.a[4], obs_.a[5];
      bool res = robot_.fcl_separation(robot_, obs_, r_i_, r_j_, robot_.a[2], obs_.a[2]);
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
      bool aux = robot_.fcl_separation(robot_, arena_, r_i_, r_j_, robot_.a[2], arena_.a[2]);
      if(aux == true){
        res = false;
      }       
      return res;
    }

    og::SimpleSetupPtr ss_;
    std::vector<SuperEllipse> arena;
    std::vector<SuperEllipse> robot;
    std::vector<SuperEllipse> obstacles;
};

int main(int argc, char ** argv){
   	if (argc < 2) {
        cerr<< "Usage: Please add configuration file name, vertices file and triangles file" << endl;
        return 1;
    }

	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    
  	std::vector<SuperEllipse> robot_parts;
	std::vector<SuperEllipse> arena_parts;
	std::vector<SuperEllipse> obstacles;
  	
	std::fstream inputfile(argv[1],std::ios_base::in);
	if(!inputfile){
       std::cout << "Error opening the file" << std::endl;
       return(1);
    }
    
	//Getting robot parts and storing them in vector robot_parts    
	int n_r_parts;
	inputfile >> n_r_parts;
	robot_parts.resize(n_r_parts);
	double p0,p1,p2,p3,p4,p5;
	int p6;
	for(int i=0; i<n_r_parts;i++){
		inputfile >> p0 >>p1 >> p2>> p3 >>p4>>p5;
		inputfile >> p6;
		robot_parts[i] = {{p0,p1,p2,p3,p4,p5}, p6};	
	}

	//Getting arena parts and storing them in vector arena_parts
	int n_a_parts;
	inputfile >> n_a_parts;
	arena_parts.resize(n_a_parts);
	for(int i=0; i<n_a_parts;i++){
		inputfile >> p0 >>p1 >> p2>> p3 >>p4>>p5;
		inputfile >> p6;		
		arena_parts[i] = {{p0,p1,p2,p3,p4,p5}, p6};	
	}

	//Getting the obstacles and storing them in vector obstacles
	int n_o_parts;
	inputfile >> n_o_parts;
	obstacles.resize(n_o_parts);
	for(int i=0; i<n_o_parts;i++){
		inputfile >> p0 >>p1 >> p2>> p3 >>p4>>p5;
		inputfile >> p6;
		obstacles[i] = {{p0,p1,p2,p3,p4,p5}, p6};	
	}

	//Getting bounderies
    double b1,b2;
    inputfile >> b1>>b2;

    //Getting start configuration
    
    std::vector<double> start;
    start.resize(3);
    start[0] = robot_parts[0].a[4];
    start[1] = robot_parts[0].a[5];
    start[2] = robot_parts[0].a[2];

    //Getting goal configuration
    double c1,c2,c3;
    inputfile >> c1 >>c2 >>c3;
    std::vector<double> goal;
    goal.resize(3);
    goal[0] = c1;
    goal[1] = c2;
    goal[2] = c3;
    inputfile.close();

    //Getting the vertices and triangles from files
    robot_parts[0].readingVerticesNTriangles(argv[2], argv[3]);  
    arena_parts[0].readingVerticesNTriangles(argv[4], argv[5]);  
    for(int i=0;i<obstacles.size();i++){
        int index = (i*2) +6;
        obstacles[i].readingVerticesNTriangles(argv[index], argv[index+1]);
    }

    //Initializing tester
    PRMtester tester(b1,b2,arena_parts, robot_parts, obstacles);
    //Planning
    tester.plan(start, goal);

    return 0;
}
