#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
class PointCloudProcess{
public:
    PointCloudProcess(){
//        curr_pub = curr_node.advertise<sensor_msgs::PointCloud2>("/registered_point_cloud",1);
        curr_sub = curr_node.subscribe("/lidar/vlp32_middle/PointCloud2", 1, &PointCloudProcess::pointCloudCallback,this);
        pcl::PointCloud<pcl::PointXYZI>::Ptr emptyCloud (new pcl::PointCloud<pcl::PointXYZI>());
        before_cloud = emptyCloud;
        full_time = ros::Duration(0);
        regis_num = 0 ;

    }


    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_msg)
    {
        /**get cloud**/
        ros::Time start = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>());



        const ros::Time stamp =  point_msg->header.stamp;
        pcl::fromROSMsg(*point_msg,*cloud);
        /**empty check**/
        if (before_cloud->empty())
        {
            before_cloud = cloud;
            ROS_INFO("initial");
            return;
        }
        /**nums output**/
        int nums = cloud->size();
        int before_nums = before_cloud->size();
        ROS_INFO("to regis cloud with %d points and %d points ",before_nums,nums);
        /**main_icp -------down Sample needed**/
        pcl::IterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(before_cloud);
        pcl::PointCloud<pcl::PointXYZI> result;
//        registration.set
        icp.align(result);

        ROS_INFO("result: %d",icp.hasConverged());
        ROS_INFO("match Score: %f",icp.getFitnessScore());
        cout<<icp.getFinalTransformation()<<endl;
        /**merge**/
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZI>());
        vector< pcl::PointXYZI, Eigen::aligned_allocator< pcl::PointXYZI > > cpoints= before_cloud->points;
        for (size_t i=0; i<before_cloud->size();i++){
            cpoints[i].intensity=2000;
        }
        before_cloud->points = cpoints;
        cpoints= result.points;
        for (size_t i=0; i<result.size();i++){
            cpoints[i].intensity=400;
        }
        result.points=cpoints;
        *merged_cloud = *before_cloud+result;
        sensor_msgs::PointCloud2 merged_message;
        pcl::toROSMsg(*merged_cloud,merged_message);
        curr_pub.publish(merged_message);
//        loop_rate->sleep();
        /**next**/
        before_cloud = cloud;
        /**time used**/
        ros::Duration time_collapsed  =ros::Time::now() - start;
        ROS_INFO("regis cloud in sec:%d nsec:%d ",time_collapsed.sec,time_collapsed.nsec);
        regis_num++;
        full_time+=time_collapsed;
        average_regis_time = ((double_t)full_time.sec/(double_t)regis_num*(double_t)1e9+(double_t)full_time.nsec/(double_t)regis_num)/1e6;
        ROS_INFO("average_regis_time_is: %f ms",average_regis_time);



    }
private:
    ros::NodeHandle curr_node;
    ros::Publisher curr_pub;
    ros::Subscriber curr_sub;
//    ros::Rate *loop_rate = new ros::Rate(5);
    pcl::PointCloud<pcl::PointXYZI>::Ptr before_cloud ;
    ros::Duration full_time;
    double_t average_regis_time;
    int regis_num;


};




int main(int argc, char *argv[])
{

    ros::init(argc, argv, "register");

    PointCloudProcess pointCloudProcess;

    ros::spin();

    return 0;
}