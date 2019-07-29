#include "lidar_odometry.h"
#include "points_downsampler.h"


LidarOdometry::LidarOdometry() {
    last_scan.reset(new pcl::PointCloud<pcl::PointXYZI>());
    current_scan.reset(new pcl::PointCloud<pcl::PointXYZI>());
    aligned_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ndt_threads = 16;
    ndt_resolution = 1.0;
    // duration = 0.0;
    odom_matrix = Eigen::Matrix4f::Identity();
}

void LidarOdometry::pointsCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    clock_t start, end;
    start = clock();
    nav_msgs::Odometry stamp1;
    stamp1.header.stamp = points_msg->header.stamp;
    stamp_pub.publish(stamp1);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr removed_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pass_through(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZI>());//M
    pcl::fromROSMsg(*points_msg, *points);

    *removed_points = removePointsByRange(*points, 3, 30);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (removed_points);            
    pass.setFilterFieldName ("z");         
    pass.setFilterLimits (-2.0, 0.5);     
    //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
    pass.filter (*pass_through);

    pcl::VoxelGrid<pcl::PointXYZI> grid;
    grid.setInputCloud(pass_through);
    grid.setLeafSize(0.1, 0.1, 0.1);
    grid.filter(*grid_points);
    
    if(last_scan->points.size() == 0) {
        *last_scan = *grid_points;
        // last_timestamp = points_msg->header.stamp.sec;
    }
    else {
        *current_scan = *grid_points;
        // current_timestamp = points_msg->header.stamp;
    }

    if(last_scan->points.size() != 0 && current_scan->points.size() != 0) {
        odom_matrix = matching();
        nav_msgs::Odometry odom;
        odom.header.stamp = points_msg->header.stamp;
        odom.header.frame_id = "lidar";

        odom.pose.pose.position.x = odom_matrix(0, 3);
        odom.pose.pose.position.y = odom_matrix(1, 3);
        odom.pose.pose.position.z = odom_matrix(2, 3);

        Eigen::Matrix3f rotation_matrix;
        rotation_matrix = odom_matrix.block<3,3>(0,0);
        Eigen::Quaternionf odom_quat(rotation_matrix);
        odom.pose.pose.orientation.w = odom_quat.w();
        odom.pose.pose.orientation.x = odom_quat.x();
        odom.pose.pose.orientation.y = odom_quat.y();
        odom.pose.pose.orientation.z = odom_quat.z();

        // odom.child_frame_id = "lidar";

        end = clock();
        double a1 = end-start;
        double a2 = CLOCKS_PER_SEC;
        double duration = a1/a2*1e9;
        cout << "duration: " << duration << endl;
        odom.twist.twist.linear.x = points_msg->header.stamp.sec; 
        odom.twist.twist.linear.y = points_msg->header.stamp.nsec + duration;
        odom.twist.twist.angular.z = 0.0;

        cout << "second: " << odom.twist.twist.linear.x << endl;
        cout << "nsecond: " << odom.twist.twist.linear.y << endl;

        cout << "delta second: " << odom.twist.twist.linear.x -  points_msg->header.stamp.sec<< endl;
        cout << "delta nsecond: " << (odom.twist.twist.linear.y - points_msg->header.stamp.nsec)/1e9<< endl;

        odom_pub.publish(odom);
        points_pub.publish(points_msg);
        cout << "last_scan size: " << last_scan->points.size() <<endl;
        cout << "current_scan size: " << current_scan->points.size() <<endl;
        *last_scan = *current_scan;
        // last_timestamp = current_timestamp;
        
        cout << "odom: " << endl << odom_matrix << endl; 
    }  
}

Eigen::Matrix4f LidarOdometry::matching() {
    Eigen::Matrix4f a; // = Eigen::Matrix4f::Identity();
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt (new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>() );


    ndt->setInputSource(current_scan);
    ndt->setInputTarget(last_scan);
    ndt->setNumThreads(ndt_threads);
    ndt->setTransformationEpsilon(0.03);
    ndt->setStepSize(0.02);
    ndt->setMaximumIterations(64);
    ndt->setResolution(1.0);
    // if(nn_search_method == "KDTREE") {
    // ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    // } 
    // else if (nn_search_method == "DIRECT1") {
    // ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    // } 
    // else {
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->align(*aligned_points);
    a = ndt->getFinalTransformation();
    // }
    return a;
}