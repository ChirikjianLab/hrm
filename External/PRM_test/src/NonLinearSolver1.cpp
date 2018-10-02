#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include <unsupported/Eigen/Polynomials>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

using namespace Eigen;
using namespace std;

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
typedef _Scalar Scalar;
enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
};
typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

int m_inputs, m_values;

Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

int inputs() const { return m_inputs; }
int values() const { return m_values; }

};

struct my_functor : Functor<double>
{
my_functor(void): Functor<double>(2,2) {}
int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
    // Implement y = 10*(x0+3)^2 + (x1-5)^2
    fvec(0) = 10.0*pow(x(0)+3.0,2) +  pow(x(1)-5.0,2);
    fvec(1) = 0;

    return 0;
}
};


int main(int argc, char *argv[])
{
Eigen::VectorXd x(2);
x(0) = 2.0;
x(1) = 3.0;
std::cout << "x: " << x << std::endl;

my_functor functor;
Eigen::NumericalDiff<my_functor> numDiff(functor);
Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>,double> lm(numDiff);
lm.parameters.maxfev = 2000;
lm.parameters.xtol = 1.0e-10;
std::cout << lm.parameters.maxfev << std::endl;

int ret = lm.minimize(x);
std::cout << lm.iter << std::endl;
std::cout << ret << std::endl;

std::cout << "x that minimizes the function: " << x << std::endl;

std::cout << "press [ENTER] to continue " << std::endl;
std::cin.get();
return 0;
}

/*int main(int argc, char ** argv){
    bool res = false;
    typedef Matrix<double,5,1> Vector5d;
        
    //Solving characteristic polynomial
    Vector5d characteristic_polynomial;
    characteristic_polynomial << T0,T1,T2,T3,T4;
 
    //cout << "characteristic_polynomial: " << characteristic_polynomial.transpose() << endl;
    PolynomialSolver<double, 4>psolve;
    psolve.compute(characteristic_polynomial);
    //cout << "Complex roots: " << psolve.roots().transpose() << endl;
           
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
            res = true;
         }
         if(abs(roots(0,0) - roots(1,0))<0.001){
            res = false;
         }
     }
    return 0;
}*/

