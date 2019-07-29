#ifndef LIDARODOMETRY_H_
#define LIDARODOMETRY_H_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <cmath>  
#include <stack> 
#include <limits>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <unistd.h>
#include <sys/stat.h>
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件

#include <pcl/surface/processing.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
// #include "points_downsampler.h"

using namespace std;

class LidarOdometry {
    public:
     LidarOdometry();
     ~LidarOdometry(){}
     void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg);
     Eigen::Matrix4f matching();
     //  void odomPublish();
     ros::Publisher odom_pub, points_pub, stamp_pub;
     ros::Subscriber points_sub;
     pcl::PointCloud<pcl::PointXYZI>::Ptr last_scan;
     pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan;
     pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_points;

    private:
     
     int ndt_threads;
     double ndt_resolution;
     Eigen::Matrix4f odom_matrix; 
    //  double last_timestamp, current_timestamp, pub_timestamp;
    //  double sum_time;  
};

#endif
