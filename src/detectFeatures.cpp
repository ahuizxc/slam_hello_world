/*************************************************************************
	> File Name: detectFeatures.cpp
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
    > 特征提取与匹配
	> Created Time: 2015年07月18日 星期六 16时00分21秒
 ************************************************************************/

#include<iostream>
#include "slamBase.h"
using namespace std;
int main( int argc, char** argv )
{
    // 声明并从data文件夹里读取两个rgb与深度图
    FRAME frame1, frame2;
    frame1.rgb = cv::imread( "/home/intel/rgb2cld/data/rgb1.png");
    frame2.rgb = cv::imread( "/home/intel/rgb2cld/data/rgb2.png");
    frame1.depth = cv::imread( "/home/intel/rgb2cld/data/depth1.png", -1);
    frame2.depth = cv::imread( "/home/intel/rgb2cld/data/depth2.png", -1);
    // 声明特征提取器与描述子提取器
    computeKeyPointsAndDesp(frame1);
    computeKeyPointsAndDesp(frame2);
    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS C;
    C.cx = 325.5;
    C.cy = 253.5;
    C.fx = 518.0;
    C.fy = 519.0;
    C.scale = 1000.0;

    RESULT_OF_PNP pnp_result = estimateMotion(frame1, frame2, C);
    cout<<"inliers: "<<pnp_result.inliers.rows<<endl;
    cout<<"R="<<pnp_result.rvec<<endl;
    cout<<"t="<<pnp_result.tvec<<endl;
    return 0;
}