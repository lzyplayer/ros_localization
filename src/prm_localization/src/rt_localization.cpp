#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
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
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>




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
        /**sub and pub**/
        points_suber = nh.subscribe("/velodyne_points",1,&RealTime_Localization::points_callback,this);
        localmap_suber =  nh.subscribe("/localmap",1,&RealTime_Localization::localmap_callback,this);
        /**utility param**/
        downSampler.setLeafSize(0.01f,0.01f,0.01f);
    }

private:

    void points_callback(const sensor_msgs::PointCloud2ConstPtr points_msg){
        pcl::PointCloud<pcl::PointXY>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXY>());
        pcl::fromROSMsg(*points_msg, *curr_cloud);
        downSampler.setInputCloud(curr_cloud);
////////////////////////////////

    }
    void localmap_callback(const sensor_msgs::PointCloud2ConstPtr points_msg){
        localmap_cloud.reset(new pcl::PointCloud<pcl::PointXY>());
        pcl::fromROSMsg(*points_msg, *localmap_cloud);
        const auto& stamp = points_msg->header.stamp;
        pcl_conversions::toPCL(points_msg->header,localmap_cloud->header);

    }

private:

    ros::NodeHandle nh;
    // suber and puber
    ros::Publisher curr_pub;
    ros::Subscriber odom_suber;
    ros::Subscriber points_suber;
    ros::Subscriber localmap_suber;
    // clouds
    pcl::PointCloud<pcl::PointXY>::Ptr localmap_cloud;

    // utility
    pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> downSampler;
    // time log
    ros::Duration full_time;
    double_t average_regis_time;
    int regis_num;


};




int main(int argc, char *argv[])
{

    ros::init(argc, argv, "rt_locator");
    unique_ptr<RealTime_Localization> realtime_localization ;//(new RealTime_Localization())
    realtime_localization->onInit();

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