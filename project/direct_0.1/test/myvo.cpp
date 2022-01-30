//this is test code for using Direct Methods
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#include <mutex>
#include <vector>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"


int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    myslam::Config::setParameterFile ( argv[1] );
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );
    myslam::Camera::Ptr camera ( new myslam::Camera ); 


    string associate_dir = myslam::Config::get<string> ( "associate_dir" );
    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"associate: "<<associate_dir<<endl;
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( associate_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    int i = 0;
    while (!fin.eof())
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;

        if ( fin.good() == false ){
            cout << "break while write associate"<<endl;
            break;
        }

        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );
    }
    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;

    std::mutex mView, mWork;
    std::vector<std::thread> mpts;

    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 1.0 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

    std::ofstream writeFile;
    writeFile.open("mytraj.txt");

    for ( int i=0; i<rgb_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr ){
            cout<<rgb_files[i]<<endl;
            break;
        }
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        cv::cvtColor(color,pFrame->gray_,CV_RGB2GRAY);
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];
        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST ){
            cout<<"lost"<<endl;
            break;
        }
        SE3d Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotationMatrix() ( 0,0 ), Twc.rotationMatrix() ( 0,1 ), Twc.rotationMatrix() ( 0,2 ),
                Twc.rotationMatrix() ( 1,0 ), Twc.rotationMatrix() ( 1,1 ), Twc.rotationMatrix() ( 1,2 ),
                Twc.rotationMatrix() ( 2,0 ), Twc.rotationMatrix() ( 2,1 ), Twc.rotationMatrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        Eigen::Quaterniond q = Eigen::Quaterniond (Twc.rotationMatrix());
        string traj = to_string(rgb_times[i]) + " " + to_string(Twc.translation()(0,0)) + " " 
                    + to_string(Twc.translation()(1,0)) + " " + to_string(Twc.translation()(2,0)) + " "
                    + to_string(q.coeffs()(0,0)) + " " + to_string(q.coeffs()(1,0)) + " " 
                    + to_string(q.coeffs()(2,0)) + " " + to_string(q.coeffs()(3,0)) + "\n"; 
        if(writeFile.is_open()){
            writeFile.write(traj.c_str(), traj.size());
        }

        Mat img_show = color.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            ushort d = pFrame->depth_.ptr<ushort> (int(pixel(1,0)))[int(pixel(0,0))];
            cv::circle ( img_show, cv::Point2f(pixel(0,0),pixel(1,0)), 2, cv::Scalar(255), 1 );
        }
        cv::imshow ( "image", img_show );
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        cv::waitKey ( 100 );
        cout<<endl;
    }
    writeFile.close();
    return 0;
}
