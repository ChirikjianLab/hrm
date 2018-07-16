#include <ompl/base/spaces/RealVectorStateSpace.h>
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
    bool algebraic_separation_condition(Vector2d coeff_canon_i_, Vector2d coeff_canon_j_, Vector2d r_i_, Vector2d r_j_, Matrix2d A_i_, Matrix2d A_j_){       
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
        PolynomialSolver<double,6> psolve( characteristic_polynomial );
        //cout << "Complex roots: " << psolve.roots().transpose() << endl;
        
        /*Checking roots conditions
        * Two different negative real roots : separated
        * If real part is =, then they touch in one point
        * If there are more or less than 2 negative real roots, then they touch.
        */
        
        MatrixXf roots(6, 1);
        roots << 0,0,0,0,0,0;
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
      S << 0, 1, -1,0;
      Matrix2d R;
      MatrixXd id = MatrixXd::Identity(2,2);
      R = id + ((sin(theta)) * S) + ((1 - cos(theta))*(S * S));
      return R;
    }
};


class PRMtester{
  public:
    PRMtester(double lowBound, double highBound, SuperEllipse arena_, SuperEllipse robot_, std::vector<SuperEllipse> obs_){        
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
        //Saving the environment configuration
        ofstream file_conf;
        file_conf.open("configuration.csv");
        file_conf << robot.a[0] << "," << robot.a[1] << "," << robot.a[2] << "," << robot.a[3] << "," <<robot.a[4] << "," <<robot.a[5] << "\n";
        file_conf << arena.a[0] << "," << arena.a[1] << "," << arena.a[2] << "," << arena.a[3] << "," <<arena.a[4] << "," <<arena.a[5] << "\n";
        for(unsigned int j=0; j<obstacles.size();j++){
          file_conf << obstacles[j].a[0] << "," << obstacles[j].a[1] << "," << obstacles[j].a[2] << "," << obstacles[j].a[3] << "," <<obstacles[j].a[4] << "," <<obstacles[j].a[5] << "\n";
        }
        file_conf.close();
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

        SuperEllipse robot_config = {{robot.a[0],robot.a[1],yaw,robot.a[3],x,y}, robot.num};
        
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
        return checkASCArena(robot_config, arena);
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
      Matrix2d A_i_ = robot_.rotation_angle_axis(robot_.a[2]); 
      Matrix2d A_j_ = arena_.rotation_angle_axis(arena_.a[2]);
      bool aux = robot_.algebraic_separation_condition(coeff_canon_i_, coeff_canon_j_, r_i_, r_j_, A_i_, A_j_);
      if(aux == true){
        res = false;
      }       
      return res;
    }

    og::SimpleSetupPtr ss_;
    SuperEllipse arena;
    SuperEllipse robot;
    std::vector<SuperEllipse> obstacles;
};

int main(int argc, char ** argv){
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    int num = 50;

    SuperEllipse robot = {{5,3,0,1.0,0,0}, num};
    SuperEllipse arena = {{50,30,0,1.0,0,0}, num};
    std::vector<SuperEllipse> obs = { {{20,10,pi/4,1.0,20,0}, num}, {{10,8,0,1.0,-20,10}, num} };

    PRMtester tester(-60.0,60.0,arena, robot, obs);
    std::vector<double> start;
    start.resize(3);
    start[0] = -20.0;
    start[1] = -10.0;
    start[2] = 0.0;

    std::vector<double> goal;
    goal.resize(3);
    goal[0] = 20.0;
    goal[1] = 20.0;
    goal[2] = pi/4;
    
    tester.plan(start, goal);

    return 0;
}
