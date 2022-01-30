#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
namespace myslam 
{
class JacobianAccumulator {
public:
    JacobianAccumulator(
        const Mat &img1_,
        const Mat &img2_,
        const vector<cv::KeyPoint> pixel_ref_,
        const vector<double> depth_ref_,
        Sophus::SE3d &T21_,
        Camera::Ptr intrinsic_ ):
        img1(img1_), img2(img2_), pixel_ref(pixel_ref_), depth_ref(depth_ref_), T21(T21_), intrinsic(intrinsic_){}

    void accumulate_jacobian(const cv::Range &range);
    Matrix6d hessian() const { return H; }
    Vector6d bias() const { return b; }
    double cost_func() const { return cost; }
    float GetPixelValue(const cv::Mat &img, float x, float y);

    void reset() {
        H = Matrix6d::Zero();
        b = Vector6d::Zero();
        cost = 0;
    }

private:
    const Mat &img1;
    const Mat &img2;
    const vector<cv::KeyPoint> pixel_ref;
    const vector<double> depth_ref;
    Sophus::SE3d &T21;
    Camera::Ptr intrinsic;
    Matrix6d H = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};
}
#endif // VISUALODOMETRY_H