#include "lidar_odometry.h"



int main(int argc, char** argv) {

    LidarOdometry lidar_odom;

    ros::init(argc, argv, "mapping");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/lidar_odom", 1000);

    lidar_odom.points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar/vlp32_middle/PointCloud2",1000, &LidarOdometry::pointsCallback, &lidar_odom);

    lidar_odom.odom_pub = nh.advertise<nav_msgs::Odometry>("/lidar_odom", 32);
    lidar_odom.stamp_pub = nh.advertise<nav_msgs::Odometry>("/lidar_stamp", 32);
    lidar_odom.points_pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_points", 32);


    // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub3(nh,"/point_cloud_map", 1000);
    // message_filters::Subscriber<nav_msgs::Odometry> ins_sub(nh, "/pioneer_sensors/EKF_Localization_RS232/filteredodometry", 1000);

    // typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), cloud_sub, imu_sub);
    // sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
    return 0;
}