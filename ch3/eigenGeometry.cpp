#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

///eigen 라이브러리 회전에 대한 표현 예제

int main ( int argc, char** argv )
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v ( 1,0,0 );
    cout .precision(3); 

/// Ex1 Rotation Vecter
    cout << "****Ex 1****" << endl;
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) ); 
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout<<"(1,0,0) after rotation vector = "<<v_rotated.transpose()<<endl<<endl;

/// Ex2 Rotation Matrix
    cout << "****Ex 2****" << endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    cout<<"rotation matrix =\n"<<rotation_matrix <<endl;  
    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation matrix = "<<v_rotated.transpose()<<endl<<endl;

/// Ex3  Euler angle
    cout << "****Ex 3****" << endl;
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2,1,0 );
    cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl<<endl;

/// Ex4 SO3 (Transformation Matrix)
    cout << "****Ex 4****" << endl;
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();           
    T.rotate ( rotation_vector );                                     
    T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );                     
    cout << "Transform matrix = \n" << T.matrix() <<endl;
    Eigen::Vector3d v_transformed = T*v;                         
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl<<endl;

/// Ex5 Quaternion
    cout << "****Ex 5****" << endl;
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    cout<<"quaternion from Rotation vector = \n"<<q.coeffs() <<endl;
    q = Eigen::Quaterniond ( rotation_matrix );
    cout<<"quaternion from Rotation Matrix = \n"<<q.coeffs()(2,0) <<endl;
    v_rotated = q*v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl<<endl;

    return 0;
}
