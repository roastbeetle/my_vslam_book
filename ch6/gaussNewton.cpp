#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

/// 가우스 뉴턴 예제
int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;   // parameter_real
  double ae = 2.0, be = -1.0, ce = 5.0; // parameter_estimation
  int N = 100;    
  double w_sigma = 1.0;
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;   

  vector<double> x_data, y_data;
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);                                                                // x
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));  // y = ax^2+bx+c +sigma (real)
  }

  int iterations = 100;
  double cost = 0, lastCost = 0;

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (int iter = 0; iter < iterations; iter++) {

    // H*dx = g 
    Matrix3d H = Matrix3d::Zero(); // H = JJ^T
    Vector3d g = Vector3d::Zero(); // g = -Jf
    cost = 0;

    for (int i = 0; i < N; i++) {
      double xi = x_data[i], yi = y_data[i];
      double error = yi - exp(ae * xi * xi + be * xi + ce); // error function f(x) = y_real - y_estimation
      Vector3d J; 
      J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);   // Jacobian[0] : de/da
      J[1] = -xi * exp(ae * xi * xi + be * xi + ce);        // Jacobian[1] : de/db
      J[2] = -exp(ae * xi * xi + be * xi + ce);             // Jacobian[2] : de/dc

      H += inv_sigma * inv_sigma * J * J.transpose(); // H = JJ^T
      g += -inv_sigma * inv_sigma * error * J;        // g = -fJ

      cost += error * error;
    }

    Vector3d dx = H.ldlt().solve(g);  // solve H*dx = g
    if (isnan(dx[0])) {
      cout << "result is nan!" << endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
      break;
    }

    ae += dx[0];
    be += dx[1];
    ce += dx[2];

    lastCost = cost;

    cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<
         "\t\testimated params: " << ae << "," << be << "," << ce << endl;
  }

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
  return 0;
}
