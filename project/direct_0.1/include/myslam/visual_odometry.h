#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"

namespace myslam 
{
    
struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};

class VisualOdometry
{
public:
    /// useful typedef
    typedef shared_ptr<VisualOdometry> Ptr;
    
    /// state check
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState     state_;     // current VO status
    Map::Ptr    map_;       // map with all frames and map points
    
    Frame::Ptr  ref_;       // reference key-frame 
    Frame::Ptr  cur_;      // current frame 
    
    vector<Measurement>     measurements_;
    //Mat                     descriptors_cur_;  // descriptor in current frame 
    
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
    vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points 
    vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr)
   
    SE3d T_estimated_;    // the estimated pose of current frame 
    int num_inliers_;        // number of inlier features in icp
    int num_lost_;           // number of lost times
    
    // parameters 
    int kp_min_norm_;   // number of features
    int max_num_lost_;      // max number of continuous lost times
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    double false_norm_ ;

public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    
protected:  
    // inner operation 
    void extractKeyPoints();
    void poseEstimation(); 
    void addKeyFrame();
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
};
}

#endif // VISUALODOMETRY_H