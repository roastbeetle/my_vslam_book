#include <algorithm>
#include <boost/timer.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

using namespace cv;
namespace myslam
{

VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), cur_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{
    kp_min_norm_        = Config::get<int> ( "kp_min_norm_" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    false_norm_         = Config::get<double> ( "false_norm" );
}

VisualOdometry::~VisualOdometry()
{

}

bool VisualOdometry::addFrame ( Frame::Ptr frame ){
    switch ( state_ )
    {
    case INITIALIZING:
    {
        // Initialize 
        cur_ = ref_ = frame;
        extractKeyPoints();
        addKeyFrame();
        state_ = OK;
        break;
    }
    case OK:
    {
        cur_ = frame;
        poseEstimation();
        if ( checkEstimatedPose() == true ){
            cur_->T_c_w_ = T_estimated_;
            cout<<"T ="<<T_estimated_.matrix()<<endl;
            num_lost_ = 0;
            //if ( checkKeyFrame() == true ){
            extractKeyPoints();
            addKeyFrame();
            //}
            ref_ = frame;
        }
        else{
            num_lost_++;
            if ( num_lost_ > max_num_lost_ ){
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }
    return true;
}

void VisualOdometry::extractKeyPoints(){
    measurements_.clear();
    boost::timer timer;
    std::vector<KeyPoint> keypoints;
    cv::Ptr<FeatureDetector> detector = ORB::create(1000,1.2f,4,15,0,2,ORB::HARRIS_SCORE,31,9);
    detector->detect(ref_->gray_, keypoints);
    /*
    for (int x=5; x<ref_->gray_.cols-5; x++)
        for(int y=5; y<ref_->gray_.rows-5; y++){
            Eigen::Vector2d delta(
                ref_->gray_.ptr<uchar>(y)[x+1] - ref_->gray_.ptr<uchar>(y)[x-1], 
                ref_->gray_.ptr<uchar>(y+1)[x] - ref_->gray_.ptr<uchar>(y-1)[x]
            );
            if(delta.norm()<kp_min_norm_)
                continue;
            ushort d = ref_->depth_.ptr<ushort> (y)[x];
            if(d==0)
                continue;
            Eigen::Vector3d p3d = ref_->camera_->pixel2world(Vector2d(x,y),ref_->T_c_w_,d);
            float grayscale = float(ref_->gray_.ptr<uchar>(y)[x]);
            measurements_.push_back(Measurement(p3d,grayscale));
        }
    */
    for(int i =0; i<keypoints.size(); i++){
        int x = keypoints[i].pt.x;
        int y = keypoints[i].pt.y;
        ushort d = ref_->depth_.ptr<ushort> (y)[x];
        if(d==0)
            continue;
        Eigen::Vector3d p3d = ref_->camera_->pixel2world(Vector2d(x,y),ref_->T_c_w_,d);
        float grayscale = float(ref_->gray_.ptr<uchar>(y)[x]);
        measurements_.push_back(Measurement(p3d,grayscale));  
    }

    cout<<"add total "<<measurements_.size()<<" measurements."<<endl;
    cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}

void VisualOdometry::poseEstimation(){
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm (solver);
    optimizer.setVerbose(true);

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate(g2o::SE3Quat(T_estimated_.rotationMatrix(), T_estimated_.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);


    int id=1;
    for(Measurement m: measurements_){
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect(
            m.pos_world, ref_->camera_->fx_, ref_->camera_->fy_ ,ref_->camera_->cx_, ref_->camera_->cy_, &cur_->gray_);
        edge->setVertex(0,pose);
        edge->setMeasurement(m.grayscale);
        edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
        edge->setId (id++);
        optimizer.addEdge(edge);
    }
    cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    T_estimated_=SE3d(pose->estimate().rotation(),pose->estimate().translation());
}

bool VisualOdometry::checkEstimatedPose(){
    SE3d T_r_c = ref_->T_c_w_*T_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if ( d.norm() > false_norm_ ){
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    } 
    return true;
}

bool VisualOdometry::checkKeyFrame(){
    SE3d T_r_c = ref_->T_c_w_*T_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    cout<<"rot:"<<rot.norm()<< ", trans:"<< trans.norm()<<endl;
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame(){
    map_->map_points_.clear();
    for( size_t i=0; i<measurements_.size(); i++ ){
        Vector3d n = measurements_[i].pos_world - cur_->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(measurements_[i].pos_world, n, cur_.get());
        map_->insertMapPoint( map_point );
    }
    ref_ = cur_;
}


}