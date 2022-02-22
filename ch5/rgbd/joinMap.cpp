#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>  // for formating strings
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>

using namespace std;

/// rgbd 이미지 -> 포인트 클라우드 생성 예제

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

int main(int argc, char **argv) {
    vector<cv::Mat> colorImgs, depthImgs;
    TrajectoryType poses;

    ifstream fin("./../pose.txt");
    if (!fin) {
        cerr << "check pose.txt file" << endl;
        return 1;
    }

    /// get data set
    for (int i = 0; i < 5; i++) {
        boost::format fmt("./../%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(),-1));

        double data[7] = {0};
        for (auto &d:data)
            fin >> d;
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                          Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose);
    }

    /// intrinsic
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    Eigen::Matrix<double,3,3> K;
    K << fx,0,cx,0,fy,cy,0,0,1;
    double depthScale = 1000.0;
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);

    for (int i = 0; i < 5; i++) {
        cout << "converting the image.. : " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        /// T_wc : Pose (카메라 좌표계 -> 월드 좌표계)
        Sophus::SE3d T = poses[i];  
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u];
                if (d == 0) continue; 
                Eigen::Vector3d point; // P_c
                
                /// 행렬계산
                Eigen::Vector3d pixel(u,v,1); // pixel
                point = K.inverse()*pixel*double(d)/depthScale; // P_c = Z*K^(-1)*p
                 
                /// 직접계산
                ///point[2] = double(d) / depthScale;      // Z
                ///point[0] = (u - cx) * point[2] / fx;    // X = (u-cx)*fx/Z
                ///point[1] = (v - cy) * point[2] / fy;    // y = (v-cy)*fy/Z
                
                Eigen::Vector3d pointWorld = T * point; // P_w = T*P_c
                
                /// p : < x, y, z, b, g, r >
                Vector6d p;
                p.head<3>() = pointWorld;
                p[5] = color.data[v * color.step + u * color.channels()];       // blue
                p[4] = color.data[v * color.step + u * color.channels() + 1];   // green
                p[3] = color.data[v * color.step + u * color.channels() + 2];   // red
                pointcloud.push_back(p);
            }
    }

    cout << "Distribute " << pointcloud.size() << " point clouds..." << endl;
    showPointCloud(pointcloud);
    return 0;
}

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
