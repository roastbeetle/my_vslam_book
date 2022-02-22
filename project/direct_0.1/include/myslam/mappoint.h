#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam{

class Frame;
class MapPoint{
public:
    /// member variables
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id_;
    static unsigned long factory_id_;
    bool good_;
    Vector3d pos_;
    Vector3d norm_;

    list<Frame*> observed_frames_;

    int matched_times_;
    int visible_times_;

    MapPoint();
    MapPoint(
        unsigned long id,
        const Vector3d& position,
        const Vector3d& norm,
        Frame* frame=nullptr
    );
    
    inline cv::Point3f getPositionCV() const{
        return cv::Point3f(pos_(0,0),pos_(1,0),pos_(2,0));
    }

    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint(
        const Vector3d& pos_world,
        const Vector3d& norm_,
        Frame* Frame
    );
    

};
}

#endif