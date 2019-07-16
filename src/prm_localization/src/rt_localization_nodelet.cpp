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
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//eigen
#include <Eigen/Dense>
//tf
#include <tf/transform_datatypes.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
//cpp
#include <ctime>
#include <limits.h>
#include <mutex>
//untility
#include <prm_localization/transform_utility.hpp>


namespace rt_localization_ns{
    using namespace std;


    class RealTime_Localization: public nodelet::Nodelet {
    public:
        RealTime_Localization(){
        }
        virtual  ~RealTime_Localization(){
        }

        void onInit() override {
            /**init**/
            nh = getNodeHandle();
            mt_nh = getMTNodeHandle();
            private_nh = getPrivateNodeHandle();
            /**parameter**/
            trim_low = private_nh.param<float>("trim_low", 0.0f);
            trim_high = private_nh.param<float>("trim_high", 4.0f);
            auto init_x = private_nh.param<float>("init_x", 0.0f);
            auto init_y = private_nh.param<float>("init_y", 0.0f);
            auto init_yaw = private_nh.param<double>("init_yaw", 0.0);
            auto icpMaximumIterations = private_nh.param<int>("icpMaximumIterations", 50);
            auto icpRANSACOutlierRejectionThreshold = private_nh.param<float>("icpRANSACOutlierRejectionThreshold", 0.04f);
            auto icpTransformationEpsilon = private_nh.param<float>("icpTransformationEpsilon", 1e-8);
            auto downSampleSize = private_nh.param<float>("downSampleSize", 0.05f);
            map_tf = private_nh.param<std::string>("map_tf", "map");
            base_lidar_tf = private_nh.param<std::string>("base_lidar_tf", "velodyne");


            //init_pose
            curr_pose.setIdentity(4,4);
            curr_pose.block(0,0,3,3) = euler2rot(0,0,init_yaw);
            curr_pose(0,3) =init_x;
            curr_pose(1,3) =init_y;
            curr_pose_stamp =ros::Time(0.001);
            //ndt_omp
//            pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
//            ndt->setTransformationEpsilon(0.01);
//            ndt->setResolution(1.0);
//            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
//            registration = ndt;
            //registration-icp
            registration.setMaximumIterations(icpMaximumIterations);
            registration.setRANSACOutlierRejectionThreshold(icpRANSACOutlierRejectionThreshold);
            registration.setMaxCorrespondenceDistance(icpRANSACOutlierRejectionThreshold*100);
            registration.setTransformationEpsilon(icpTransformationEpsilon);
//        registration.set
            /**sub and pub**/
//            odom_pub = nh.advertise<nav_msgs::Odometry>(map_tf,50);
            odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",50);
            get_pmsg_pub = nh.advertise<nav_msgs::Odometry>("/stamp",5);
            points_suber = nh.subscribe("/velodyne_points",1,&RealTime_Localization::points_callback,this);
            localmap_suber =  mt_nh.subscribe("/localmap",1,&RealTime_Localization::localmap_callback,this);
            curr_pointcloud_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/registered_pointCloud",5);
            /**utility param**/
            downSampler.setLeafSize(downSampleSize,downSampleSize,downSampleSize);

            NODELET_INFO("realTime_localization_nodelet initial completed");
        }

    private:

        void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg){
            //send get flag (optional)
            nav_msgs::Odometry flag_odom;
            flag_odom.header.stamp=points_msg->header.stamp;
            get_pmsg_pub.publish(flag_odom);
            // convert ros msg
            pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*points_msg, *curr_cloud);
            //downsample
            downSampler.setInputCloud(curr_cloud);
            downSampler.filter(*filtered_cloud);
            //trim and 2d
            auto flat_cloud  = trimInputCloud(filtered_cloud, trim_low, trim_high);
            //pc register
            Matrix4f transform = pc_register(flat_cloud,localmap_cloud,curr_pose);
            {
                lock_guard<mutex> lockGuard(curr_pose_mutex);
                if(points_msg->header.stamp>curr_pose_stamp){
                    curr_pose=transform;
                    NODELET_INFO("time to former stamp = %f seconds",points_msg->header.stamp.toSec()-curr_pose_stamp.toSec());
                    curr_pose_stamp = points_msg->header.stamp;

                } else {
                    NODELET_INFO("abandon former regis result");
                    return;
                }
            }

            //publish tf
            transformBroadcaster.sendTransform(matrix2transform(points_msg->header.stamp,curr_pose,map_tf,base_lidar_tf));
            //publish odom (optional)
            nav_msgs::Odometry odom;
            odom.header.stamp=points_msg->header.stamp;
            odom.header.frame_id=map_tf;
            odom.child_frame_id=base_lidar_tf;
            odom.pose.pose.position.x=transform(0,3);
            odom.pose.pose.position.y=transform(1,3);
            odom.pose.pose.position.z=transform(2,3);
            Eigen::Quaternionf q = rot2quat(transform);
            odom.pose.pose.orientation.x=q.x();
            odom.pose.pose.orientation.y=q.y();
            odom.pose.pose.orientation.z=q.z();
            odom.pose.pose.orientation.w=q.w();
            odom_pub.publish(odom);

            //publish cloud (optional)
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coloredCloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::RGB>::Ptr color (new pcl::PointCloud<pcl::RGB>());
            pcl::transformPointCloud(*curr_cloud,*transformedCloud,transform);
            color->width = transformedCloud->width;
            color->height = 1;//result_cloud.height
            color->points.resize(color->width*color->height);
            for (size_t i=0;i<transformedCloud->width;i++){
                color->points[i].r=255;
                color->points[i].g=255;
                color->points[i].a=100;
            }
            pcl::concatenateFields(*transformedCloud,*color,*coloredCloud);

            pcl_conversions::toPCL(points_msg->header,coloredCloud->header);
            coloredCloud->header.frame_id = map_tf;
            curr_pointcloud_pub.publish(coloredCloud);



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
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTrimmed (new pcl::PointCloud<pcl::PointXYZ>());
            for(size_t i=0;i<cloud->points.size();++i){
                if (cloud->points[i].z<high && cloud->points[i].z>low) {
                    pcl::PointXYZ currPoint;
                    currPoint.x =cloud->points[i].x;
                    currPoint.y =cloud->points[i].y;
                    //2d
//                currPoint.z =0;
                    //3d
                    currPoint.z =cloud->points[i].z;
                    cloudTrimmed->points.push_back(currPoint);
                }
            }
            cloudTrimmed->width = cloudTrimmed->points.size();
            cloudTrimmed->header = cloud->header;
            return cloudTrimmed;
        }

        Eigen::Matrix4f pc_register(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& curr_cloud,const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& local_map,const Eigen::Matrix4f& initial_matrix){
            //registration
            registration.setInputSource(curr_cloud);
            registration.setInputTarget(local_map);
            pcl::PointCloud<pcl::PointXYZ> result_cloud ;
            //registration start
            clock_t start = clock();
            registration.align(result_cloud,curr_pose);//,curr_pose
            clock_t end = clock();
            NODELET_INFO("registration regis time = %f seconds",(double)(end  - start) / CLOCKS_PER_SEC);
//            NODELET_INFO("fitness score = %f ",registration.getFitnessScore());
//            NODELET_INFO(" ");
            return registration.getFinalTransformation();
//        Vector3f euler = rot2euler(transform.block(0,0,3,3));
//        NODELET_INFO("x = %f, y = %f, theta = %f ",transform(0,3),transform(1,3),euler(2));
        }

    private:

        //ros node handle
        ros::NodeHandle nh;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;
        // para
        float trim_low;
        float trim_high;
        Eigen::Matrix4f curr_pose;
        ros::Time curr_pose_stamp;
        // suber and puber
        ros::Publisher curr_pointcloud_pub;
        ros::Subscriber odom_suber;
        ros::Subscriber points_suber;
        ros::Subscriber localmap_suber;
        ros::Publisher odom_pub;
        ros::Publisher get_pmsg_pub;
        ros::Timer timer;
        tf::TransformBroadcaster transformBroadcaster;
        //tf
        string map_tf;
        string base_lidar_tf;

        // clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr localmap_cloud;

        // utility
//        pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
        pcl::VoxelGrid<pcl::PointXYZ> downSampler;
        std::mutex curr_pose_mutex;
        // time log
        ros::Duration full_time;
        double_t average_regis_time;
        int regis_num;


    };


}
PLUGINLIB_EXPORT_CLASS(rt_localization_ns::RealTime_Localization, nodelet::Nodelet)

//                NODELET_INFO("sec:%i,nsec:%i",points_msg->header.stamp.sec,points_msg->header.stamp.nsec);
//                NODELET_INFO("sec:%i,nsec:%i",curr_pose_stamp.sec,curr_pose_stamp.nsec);


//
//int main(int argc, char *argv[])
//{
//
//    //test code
////    pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud (new pcl::PointCloud<pcl::PointXYZ>());
////    pcl::PointXYZ pointXy;
////    pointXy.x=2;
////    pointXy.y=2;
////
////    mycloud->points.push_back(pointXy);
////    //
//
//
//
//    ros::init(argc, argv, "rt_locator");
//    RealTime_Localization realtime_localization ;//(new RealTime_Localization())
//    realtime_localization.onInit();
//
//    ros::spin();
//
//    return 0;
//}
