#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//eigen
#include <Eigen/Dense>
//nodelet
//#include <nodelet/nodelet.h>
//#include <pluginlib/class_list_macros.h>
//cpp
#include <ctime>
using namespace std;


int main(int argc, char *argv[]) {
    std::string globalmap_pcd = "/home/vickylzy/WorkSPacesROS/catkin_ws/src/prm_localization/data/shunYuFactory.pcd";//private_nh.param<std::string>("globalmap_pcd", "");
    std::string currmap_pcd = "/home/vickylzy/WorkSPacesROS/catkin_ws/src/prm_localization/data/shunYu1.pcd";//private_nh.param<std::string>("globalmap_pcd", "");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile(globalmap_pcd, *full_cloud);
    full_cloud->header.frame_id = "map";
    pcl::io::loadPCDFile(currmap_pcd, *curr_cloud);
    curr_cloud->header.frame_id = "map";

    clock_t init = clock();

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputTarget(full_cloud);
    icp.setInputSource(curr_cloud);
    //para
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-7);
    icp.setRANSACOutlierRejectionThreshold(0.04);
    icp.setMaxCorrespondenceDistance(4);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
    /** analyse RANSACOutlierRejectionThreshold and MaxCorrespondenceDistance**/
    for (double i = 0.01; i <0.15 ; i+=0.01) {
        icp.setRANSACOutlierRejectionThreshold(i);
        icp.setMaxCorrespondenceDistance(i*100);
        clock_t start = clock();
        icp.align(*result);
        clock_t end = clock();
        cout<<"registration in "<< (double)(end  - start) / CLOCKS_PER_SEC << "second" << endl;
        cout<<"with"<< "registration.setRANSACOutlierRejectionThreshold() = "<< i <<endl;
        cout<<"registration.getFitnessScore() = "<<icp.getFitnessScore()<<endl;
        cout<<endl;

    }
    /** analyse TransformationEpsilon**/
    for (double i = 1e-11; i <=1e-7 ; i*=10) {
        icp.setTransformationEpsilon(i);
        clock_t start = clock();
        icp.align(*result);
        clock_t end = clock();
        cout<<"registration in "<< (double)(end  - start) / CLOCKS_PER_SEC << "second" << endl;
        cout<<"with"<< "TransformationEpsilon() = "<< i <<endl;
        cout<<"registration.getFitnessScore() = "<<icp.getFitnessScore()<<endl;
        cout<<endl;

    }
//    clock_t start = clock();

//    registration.align(*result);

//    clock_t end = clock();
    //color
//    for (size_t i = 0; i < full_cloud->points.size() ; ++i) {
//        full_cloud->points[i].r = 255;
//        full_cloud->points[i].g = 0;
//        full_cloud->points[i].b = 0;
//    }
//    for (size_t i = 0; i < result->points.size() ; ++i) {
//        result->points[i].r=255;
//        result->points[i].g=255;
//        result->points[i].b=255;
//    }
//    cout<<"init in "<< (double)(start  - init) / CLOCKS_PER_SEC << "second" << endl;
//
//    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show (new pcl::PointCloud<pcl::PointXYZRGB>);
//    *show = *result+*full_cloud;
//    viewer.showCloud(show);
//    while (!viewer.wasStopped ())
//    {
//    }








}