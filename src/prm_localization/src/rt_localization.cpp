#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>

#include <nav_msgs/Odometry.h>
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//eigen
#include <Eigen/Dense>
//tf
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
//nodelet
//#include <nodelet/nodelet.h>
//#include <pluginlib/class_list_macros.h>
//cpp
#include <ctime>
//untility
#include <prm_localization/transform_utility.hpp>



using namespace std;


class RealTime_Localization{
public:
    RealTime_Localization(){
    }
    virtual  ~RealTime_Localization(){
    }

    void onInit()  { //override
        /**parameter**/
//        nh.getParam()
        trim_low = 0.0f;
        trim_high= 4.0f;
        curr_pose.setIdentity(4,4);
        icp.setMaximumIterations(150);
        icp.setRANSACOutlierRejectionThreshold(0.04);
        icp.setMaxCorrespondenceDistance(0.04*100);
        icp.setTransformationEpsilon(1e-7);

//        icp.set
        /**sub and pub**/
//        odom_pub = nh.advertise<nav_msgs::Odometry>("velodyne_link",50);
        points_suber = nh.subscribe("/velodyne_points",1,&RealTime_Localization::points_callback,this);
        localmap_suber =  nh.subscribe("/localmap",1,&RealTime_Localization::localmap_callback,this);
        curr_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/registered_pointCloud",5);
        timer = nh.createTimer(ros::Duration(0.1),&RealTime_Localization::tfpublisher,this);
        /**utility param**/
        downSampler.setLeafSize(0.05f,0.05f,0.05f);
    }

private:

    void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*points_msg, *curr_cloud);
        //downsample
        downSampler.setInputCloud(curr_cloud);
        downSampler.filter(*filtered_cloud);
        //trim and 2d
        auto flat_cloud  = trimInputCloud(filtered_cloud, trim_low, trim_high);

        //icp
        icp.setInputTarget(localmap_cloud);
        icp.setInputSource(flat_cloud);
        pcl::PointCloud<pcl::PointXYZ> result_cloud ;
        //icp start
        clock_t start = clock();
        icp.align(result_cloud,curr_pose);//,curr_pose
        clock_t end = clock();
        ROS_INFO("icp regis time = %f seconds",(double)(end  - start) / CLOCKS_PER_SEC);
        ROS_INFO("fitness score = %f ",icp.getFitnessScore());
        Matrix4f transform = icp.getFinalTransformation();
        Vector3f euler = rot2euler(transform.block(0,0,3,3));
        ROS_INFO("x = %f, y = %f, theta = %f ",transform(0,3),transform(1,3),euler(2));
        //
        curr_pose=icp.getFinalTransformation();
        //pub (optional)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        coloredCloud->width = result_cloud.width;
        coloredCloud->height = 1;//result_cloud.height
        coloredCloud->points.resize(coloredCloud->width*coloredCloud->height);

        for (size_t i=0;i<result_cloud.width;i++){
            coloredCloud->points[i].x = result_cloud.points[i].x;
            coloredCloud->points[i].y = result_cloud.points[i].y;
            coloredCloud->points[i].z = result_cloud.points[i].z;
            coloredCloud->points[i].b=255;
        }
        //
        pcl_conversions::toPCL(points_msg->header,coloredCloud->header);
        coloredCloud->header.frame_id = "map";
        curr_pointcloud_pub.publish(coloredCloud);



    }

    void tfpublisher(const ros::TimerEvent& event){
        //warning should not be time now
        odom_broadcaster.sendTransform(matrix2transform(ros::Time::now(),curr_pose,"map","velodyne"));



    }
    void localmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg){
        localmap_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*points_msg, *localmap_cloud);
        /** already contains stamp**/
//        const auto& stamp = points_msg->header.stamp;
//        pcl_conversions::toPCL(points_msg->header,localmap_cloud->header);

    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr trimInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                                                            const float low, const float high) const {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2d (new pcl::PointCloud<pcl::PointXYZ>());
        for(size_t i=0;i<cloud->points.size();++i){
            if (cloud->points[i].z<high && cloud->points[i].z>low) {
                pcl::PointXYZ currPoint;
                currPoint.x =cloud->points[i].x;
                currPoint.y =cloud->points[i].y;
                //2d
//                currPoint.z =0;
                //3d
                currPoint.z =cloud->points[i].z;
                cloud2d->points.push_back(currPoint);
            }
        }
        cloud2d->width = cloud2d->points.size();
        cloud2d->header = cloud->header;
        return cloud2d;
    }

        Eigen::Matrix3d pc_register(const pcl::PointCloud<pcl::PointXY>::ConstPtr& curr_cloud,const pcl::PointCloud<pcl::PointXY>::ConstPtr& local_map,const Eigen::Matrix3d& initial_matrix){

    }

private:

    ros::NodeHandle nh;
    // para
    float trim_low;
    float trim_high;
    Eigen::Matrix4f curr_pose;
    // suber and puber
    ros::Publisher curr_pointcloud_pub;
    ros::Subscriber odom_suber;
    ros::Subscriber points_suber;
    ros::Subscriber localmap_suber;
//    ros::Publisher odom_pub;
    ros::Timer timer;
    tf::TransformBroadcaster odom_broadcaster;
    // clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr localmap_cloud;

    // utility
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::VoxelGrid<pcl::PointXYZ> downSampler;
    // time log
    ros::Duration full_time;
    double_t average_regis_time;
    int regis_num;


};




int main(int argc, char *argv[])
{

    //test code
//    pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud (new pcl::PointCloud<pcl::PointXYZ>());
//    pcl::PointXYZ pointXy;
//    pointXy.x=2;
//    pointXy.y=2;
//
//    mycloud->points.push_back(pointXy);
//    //



    ros::init(argc, argv, "rt_locator");
    RealTime_Localization realtime_localization ;//(new RealTime_Localization())
    realtime_localization.onInit();

    ros::spin();

    return 0;
}


/**
 * The ros::init() function needs to see argc and argv so that it can perform
 * any ROS arguments and name remapping that were provided at the command line.
 * For programmatic remappings you can use a different version of init() which takes
 * remappings directly, but for most command-line programs, passing argc and argv is
 * the easiest way to do it.  The third argument to init() is the name of the node.
 *
 * You must call one of the versions of ros::init() before using any other
 * part of the ROS system.
 */
/**
* NodeHandle is the main access point to communications with the ROS system.
* The first NodeHandle constructed will fully initialize this node, and the last
* NodeHandle destructed will close down the node.
*/
/**
* The subscribe() call is how you tell ROS that you want to receive messages
* on a given topic.  This invokes a call to the ROS
* master node, which keeps a registry of who is publishing and who
* is subscribing.  Messages are passed to a callback function, here
* called chatterCallback.  subscribe() returns a Subscriber object that you
* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
* object go out of scope, this callback will automatically be unsubscribed from
* this topic.
*
* The second parameter to the subscribe() function is the size of the message
* queue.  If messages are arriving faster than they are being processed, this
* is the number of messages that will be buffered up before beginning to throw
* away the oldest ones.
*/

/**
 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
 * callbacks will be called from within this thread (the main one).  ros::spin()
 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
 */