#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;
typedef Eigen::Matrix<double,4,1> vec4d;

// g2o 예제 ax^3+bx^2+cx+d
// Gauss_Newton method 사용


/// Class               CurveFittingVertex   
/// Type                g2o::BaseVertex  
/// In_parameter        _estimate
/// In_function         estimate() setEstimate()
/// Update              a,b,c,d 
class CurveFittingVertex : public g2o::BaseVertex<4,vec4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Reset estimation (0,0,0)
  /// setToOriginImpl: Optimizablegraph.h (Inherit of BaseVertex)
  virtual void setToOriginImpl() override {
    _estimate << 0, 0, 0, 0;
  }
  /// update estimation
  /// oplusImpl: Optimizablegraph.h (Inherit of BaseVertex)
  virtual void oplusImpl(const double *update) override {
    _estimate += vec4d(update);
  }

  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}
};

/// Class               CurveFittingEdge 
/// Type                g2o::BaseUnaryEdge 
/// In_parameter        _error _vertices
/// In_function         estimate() setEstimate()
/// Update              error
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

  /// error = exp(real) - exp(estimate)
  /// coumputError: base_fixed_sized_edge.h (Inherit of BaseUnaryEdge)
  virtual void computeError() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const vec4d abc = v->estimate();
    _error(0, 0) = _measurement - std::exp(abc(0, 0)*_x*_x*_x + abc(1, 0)*_x*_x + abc(2, 0)*_x + abc(3,0));
  }

  /// make Jacobian
  /// linearizeOplus : base_fixed_sized_edge.h (Inherit of BaseUnaryEdge)
  virtual void linearizeOplus() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const vec4d abc = v->estimate();
    double y = exp(abc[0]*_x*_x*_x + abc[1] *_x*_x + abc[2]*_x + abc[3]);
    _jacobianOplusXi[0] = -_x*_x*_x*y;  
    _jacobianOplusXi[1] = -_x*_x*y;
    _jacobianOplusXi[2] = -_x*y;
    _jacobianOplusXi[3] = -y;

  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  double _x;
};

int main(int argc, char **argv) {
  /// param_real
  double ar = 1.0, br = 2.0, cr = 1.0, dr = 3.0; 
  /// param_estimation
  double ae = 2.0, be = -1.0, ce = 5.0, de = 4.0;
  int N = 100;
  double w_sigma = 1.0;
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;

  /// Make real_dataset
  vector<double> x_data, y_data;
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar*x*x*x + br*x*x + cr*x + dr) + rng.gaussian(w_sigma * w_sigma));
  }

  /// Optimizer
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<4, 1>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
  /// Algorithm : GaussNewton
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  /// Set Optimization Graph
  CurveFittingVertex *v = new CurveFittingVertex();  
  v->setEstimate(vec4d(ae, be, ce, de));
  v->setId(0);
  optimizer.addVertex(v);

  /// Put Data in Graph
  for (int i = 0; i < N; i++) {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(y_data[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
    optimizer.addEdge(edge);
  }

  cout << "start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(15);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  vec4d abc_estimate = v->estimate();
  cout << "estimated model: " << abc_estimate.transpose() << endl;

  return 0;
}