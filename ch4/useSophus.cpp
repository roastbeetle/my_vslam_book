#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

using namespace std;
using namespace Eigen;

///4장 sophus 활용 예제

int main(int argc, char **argv){
/// Ex1 /// SO3
    cout << "****Ex 1****" << endl;
    Matrix3d R = AngleAxisd(M_PI/3, Vector3d(0,0.6,0.8)).toRotationMatrix();
    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);
    cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl << endl;

/// Ex2 /// Lie 대수 SO3 -> so3 로그매핑 
    cout << "****Ex 2****" << endl;
    Vector3d so3 = SO3_R.log();
    cout << "so3 = "<< so3.transpose() << endl; //phi
    cout << "so3 hat =\n" << Sophus::SO3d::hat(so3) << endl; //phi -> phi^
    cout << "so3 hat vee =\n" << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl << endl; //phi^ -> phi

/// Ex3 /// 섭동모델 so3
    cout << "****Ex 3****" << endl;  
    Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3)*SO3_R;
    cout << "SO3 Perturbation updated = \n" << SO3_updated.matrix() << endl << endl;

/// Ex4 /// SE3
    cout << "****Ex 4****" << endl;
    Vector3d t(1,0,0);
    Sophus::SE3d SE3_Rt(R, t);
    Sophus::SE3d SE3_qt(q, t);
    cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
    cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl << endl;

/// Ex5 /// Lie 대수 SE3 -> se3 로그매핑
    cout << "****Ex 5****" << endl;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = "<< se3.transpose() << endl; //ksi
    cout << "se3 hat =\n" << Sophus::SE3d::hat(se3) << endl; //ksi -> ksi^
    cout << "se3 hat vee =\n" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl << endl; //ksi^ -> ksi

/// Ex6 /// 섭동모델 se3
    cout << "****Ex 6****" << endl;
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 Perturbation updated = " << endl << SE3_updated.matrix() << endl << endl;

}
