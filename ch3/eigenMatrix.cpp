#include <iostream>
using namespace std;
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

/// eigen 라이브러리 Matrix 활용예제

int main(int argc, char** argv)
{
/// Ex1 /// Basic
    cout << "****Ex 1****" << endl;
    Eigen::Matrix<float,2,3> matrix_23;
    matrix_23 << 1,2,3,4,5,6;
    cout << matrix_23 << endl << endl;

/// Ex2 /// Match variable type
    cout << "****Ex 2****" << endl;
    Eigen::Vector3d v_3d;
    Eigen::Matrix<float,3,1> vd_3d;
    v_3d << 3,2,1;
    vd_3d << 3,2,1;
    Eigen::Matrix<double,2,1> result1 = matrix_23.cast<double>()*v_3d;
    cout << result1 << endl;
    Eigen::Matrix<float,2,1> result2 = matrix_23*vd_3d;
    cout << result2 << endl << endl;

/// Ex3 /// Matrix calculation
    cout << "****Ex 3****" << endl;
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;
    cout << matrix_33.transpose() << endl << endl;
    cout << matrix_33.sum() << endl << endl;
    cout << matrix_33.trace() << endl << endl;
    cout << matrix_33*10 << endl << endl;
    cout << matrix_33.inverse() << endl << endl;
    cout << matrix_33.determinant() << endl << endl;

/// Ex4 /// Eigen calculation
    cout << "****Ex 4****" << endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver (matrix_33.transpose()*matrix_33);
    cout << "Eigen values \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors \n" << eigen_solver.eigenvectors() << endl <<endl;

/// Ex5 /// Solve AX = B
    cout << "****Ex 5****" << endl;
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE > a;
    Eigen::Matrix<double, MATRIX_SIZE, 1 > b;
    a = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    b = Eigen::MatrixXd::Random(MATRIX_SIZE,1);

    clock_t time_stt = clock();
    Eigen::Matrix<double,MATRIX_SIZE,1> x1 = a.inverse()*b;
    cout <<"time inverse matrix is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< endl;
    cout << x1 << endl << endl;

    time_stt = clock();
    Eigen::Matrix<double,MATRIX_SIZE,1> x2 = a.colPivHouseholderQr().solve(b);
    cout <<"time Qr decomposition is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< endl;
    cout << x2 << endl << endl;

/// Dynamic ?? ///
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_dynamic;
    Eigen::MatrixXd matrix_x;


    return 0;
}
