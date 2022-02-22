#ifndef FRAME_H
#define FRAME_H
#include "myslam/common_include.h"
#include "myslam/camera.h"
namespace myslam{
/// Frame class
class Frame{ 
public:
    /// member variable
    typedef std::shared_ptr<Frame> Ptr; 
    unsigned long id_;
    double time_stamp_;
    SE3d T_c_w_;
    Camera::Ptr camera_;
    Mat color_, depth_, gray_;
    bool is_key_frame_;
public:
    /// constructor
    Frame(); 
    /// param_constructor
    Frame( long id, double time_stamp=0, SE3d T_c_w=SE3d(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat());
    /// destructor
    ~Frame();

    /// member function
    static Frame::Ptr createFrame();
    Vector3d getCamCenter() const;
    void setPose(const SE3d& T_c_w);
    bool isInFrame(const Vector3d& pt_world);
};
}
#endif