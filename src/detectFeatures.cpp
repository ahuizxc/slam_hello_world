/*************************************************************************
	> File Name: detectFeatures.cpp
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
    > 特征提取与匹配
	> Created Time: 2015年07月18日 星期六 16时00分21秒
 ************************************************************************/

#include<iostream>
#include "slamBase.h"
#include<opencv2/core/eigen.hpp>
#include<pcl/common/transforms.h>
#include<pcl/visualization/cloud_viewer.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;
int main( int argc, char** argv )
{
    // 声明并从data文件夹里读取两个rgb与深度图
    FRAME frame1, frame2;
    frame1.rgb = cv::imread( "/home/hui/slam_hello_world/data/rgb1.png");
    frame2.rgb = cv::imread( "/home/hui/slam_hello_world/data/rgb2.png");
    frame1.depth = cv::imread( "/home/hui/slam_hello_world/data/depth1.png", -1);
    frame2.depth = cv::imread( "/home/hui/slam_hello_world/data/depth2.png", -1);
    // 声明特征提取器与描述子提取器
    computeKeyPointsAndDesp(frame1);
    computeKeyPointsAndDesp(frame2);
    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    camera.scale = 1000.0;

    RESULT_OF_PNP pnp_result = estimateMotion(frame1, frame2, camera);
    cout<<"inliers: "<<pnp_result.inliers.rows<<endl;
    cout<<"R="<<pnp_result.rvec<<endl;
    cout<<"t="<<pnp_result.tvec<<endl;
    cv::Mat R;
    //rodrigues transfrom, xuanzhuan vector to xuanzhuan matrix

    cv::Rodrigues(pnp_result.rvec, R);
    Eigen::Matrix3d r;
    // eigen 是c++矩阵库， 将opencv的mat格式转换为矩阵类型
    cv::cv2eigen(R,r);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(pnp_result.tvec.at<double>(0,0),pnp_result.tvec.at<double>(0,1),pnp_result.tvec.at<double>(0,2));
    T = angle;
    T(0,3) = pnp_result.tvec.at<double>(0,0);
    T(1,3) = pnp_result.tvec.at<double>(0,1);
    T(2,3) = pnp_result.tvec.at<double>(0,2);
    PointCloud::Ptr cloud1 = image2PointCloud(frame1.rgb, frame1.depth, camera);
    PointCloud::Ptr cloud2 = image2PointCloud(frame2.rgb, frame2.depth, camera);
    PointCloud::Ptr output(new PointCloud());
    pcl::transformPointCloud(*cloud1, *output, T.matrix());
    *output += *cloud2;    
    pcl::io::savePCDFile("/home/hui/result.pcd", *output);
    cout<<"Final result saved."<<endl;
    pcl::visualization::CloudViewer viewer( "viewer" );
    viewer.showCloud( output );
    while( !viewer.wasStopped() )
{
   
}
    return 0;
}