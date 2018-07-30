#include "slamBase.h"

PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    PointCloud::Ptr cloud(new PointCloud);
    for(int m=0;m < depth.rows; m++)
        for(int n=0; n < depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            PointT p;
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy; 
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];
            cloud -> points.push_back(p); 
        }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

void computeKeyPointsAndDesp( FRAME& frame)
{
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    detector->detect(frame.rgb, frame.kp);
    detector->compute(frame.rgb, frame.kp, frame.desp);
    return;
}

RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data ); 
    vector< cv::DMatch > matches; 
    cv::BFMatcher matcher;
    matcher.match( frame1.desp, frame2.desp, matches );
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    cout<<"min dis = "<<minDis<<endl;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 10*minDis)
            goodMatches.push_back( matches[i] );
    }
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }
    RESULT_OF_PNP pnp_result;
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 8, 0.99, inliers );
    pnp_result.rvec = rvec;
    pnp_result.tvec = tvec;
    cout << inliers << endl;
    return pnp_result;

}