#include <iostream>
#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

/***********************************************************************************************/

// Generic functor
// See http://eigen.tuxfamily.org/index.php?title=Functors
// C++ version of a function pointer that stores meta-data about the function
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{

  // Information that tells the caller the numeric type (eg. double) and size (input / output dim)
  typedef _Scalar Scalar;
  enum { // Required by numerical differentiation module
      InputsAtCompileTime = NX,
      ValuesAtCompileTime = NY
  };

  // Tell the caller the matrix sizes associated with the input, output, and jacobian
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  // Local copy of the number of inputs
  int m_inputs, m_values;

  // Two constructors:
  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  // Get methods for users to determine function input and output dimensions
  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

};

/***********************************************************************************************/

// https://en.wikipedia.org/wiki/Test_functions_for_optimization
// Booth Function
// Implement f(x,y) = (x + 2*y -7)^2 + (2*x + y - 5)^2
struct BoothFunctor : Functor<double>
{
  // Simple constructor
  BoothFunctor(): Functor<double>(2,2) {}

  // Implementation of the objective function
  int operator()(const Eigen::VectorXd &z, Eigen::VectorXd &fvec) const {
    double x = z(0);   double y = z(1);
    /*
     * Evaluate the Booth function.
     * Important: LevenbergMarquardt is designed to work with objective functions that are a sum
     * of squared terms. The algorithm takes this into account: do not do it yourself.
     * In other words: objFun = sum(fvec(i)^2)
     */
    fvec(0) = x + 2*y - 7;
    fvec(1) = 2*x + y - 5;
    return 0;
  }
};

/***********************************************************************************************/

// https://en.wikipedia.org/wiki/Test_functions_for_optimization
// Himmelblau's Function
// Implement f(x,y) = (x^2 + y - 11)^2 + (x + y^2 - 7)^2
struct HimmelblauFunctor : Functor<double>
{
  // Simple constructor
  HimmelblauFunctor(): Functor<double>(2,2) {}

  // Implementation of the objective function
  int operator()(const Eigen::VectorXd &z, Eigen::VectorXd &fvec) const {
    double x = z(0);   double y = z(1);
    /*
     * Evaluate Himmelblau's function.
     * Important: LevenbergMarquardt is designed to work with objective functions that are a sum
     * of squared terms. The algorithm takes this into account: do not do it yourself.
     * In other words: objFun = sum(fvec(i)^2)
     */
    fvec(0) = x * x + y - 11;
    fvec(1) = x + y * y - 7;
    return 0;
  }
};

/***********************************************************************************************/

void testBoothFun() {
  std::cout << "Testing the Booth function..." << std::endl;
  Eigen::VectorXd zInit(2); zInit << 1.87, 2.032;
  std::cout << "zInit: " << zInit.transpose() << std::endl;
  Eigen::VectorXd zSoln(2); zSoln << 1.0, 3.0;
  std::cout << "zSoln: " << zSoln.transpose() << std::endl;

  BoothFunctor functor;
  Eigen::NumericalDiff<BoothFunctor> numDiff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<BoothFunctor>,double> lm(numDiff);
  lm.parameters.maxfev = 1000;
  lm.parameters.xtol = 1.0e-10;
  std::cout << "max fun eval: " << lm.parameters.maxfev << std::endl;
  std::cout << "x tol: " << lm.parameters.xtol << std::endl;

  Eigen::VectorXd z = zInit;
  int ret = lm.minimize(z);
  std::cout << "iter count: " << lm.iter << std::endl;
  std::cout << "return status: " << ret << std::endl;
  std::cout << "zSolver: " << z.transpose() << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
}

/***********************************************************************************************/

void testHimmelblauFun() {
  std::cout << "Testing the Himmelblau function..." << std::endl;
  // Eigen::VectorXd zInit(2); zInit << 0.0, 0.0;  // soln 1
  // Eigen::VectorXd zInit(2); zInit << -1, 1;  // soln 2
  // Eigen::VectorXd zInit(2); zInit << -1, -1;  // soln 3
  Eigen::VectorXd zInit(2); zInit << 1, -1;  // soln 4
  std::cout << "zInit: " << zInit.transpose() << std::endl;
  std::cout << "soln 1: [3.0, 2.0]" << std::endl;
  std::cout << "soln 2: [-2.805118, 3.131312]" << std::endl;
  std::cout << "soln 3: [-3.77931, -3.28316]" << std::endl;
  std::cout << "soln 4: [3.584428, -1.848126]" << std::endl;

  HimmelblauFunctor functor;
  Eigen::NumericalDiff<HimmelblauFunctor> numDiff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<HimmelblauFunctor>,double> lm(numDiff);
  lm.parameters.maxfev = 1000;
  lm.parameters.xtol = 1.0e-10;
  std::cout << "max fun eval: " << lm.parameters.maxfev << std::endl;
  std::cout << "x tol: " << lm.parameters.xtol << std::endl;

  Eigen::VectorXd z = zInit;
  int ret = lm.minimize(z);
  std::cout << "iter count: " << lm.iter << std::endl;
  std::cout << "return status: " << ret << std::endl;
  std::cout << "zSolver: " << z.transpose() << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
}

/***********************************************************************************************/

int main(int argc, char *argv[])
{

std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
testBoothFun();
testHimmelblauFun();
return 0;
}
