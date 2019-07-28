#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
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
#include <boost/circular_buffer.hpp>
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
            curr_step=0;
            trim_low = private_nh.param<float>("trim_low", 0.0f);
            trim_high = private_nh.param<float>("trim_high", 4.0f);
            farPointThreshold = private_nh.param<float>("farPointThreshold", 30.0f);
            nearPointThreshold = private_nh.param<float>("nearPointThreshold", 1.4f); //pioneer carself
            auto init_x = private_nh.param<float>("init_x", 0.0f);
            auto init_y = private_nh.param<float>("init_y", 0.0f);
            auto init_yaw = private_nh.param<double>("init_yaw", 0.0);
            auto icpMaximumIterations = private_nh.param<int>("icpMaximumIterations", 50);
            auto icpRANSACOutlierRejectionThreshold = private_nh.param<float>("icpRANSACOutlierRejectionThreshold", 0.04f);
            auto icpTransformationEpsilon = private_nh.param<float>("icpTransformationEpsilon", 1e-8);
            auto downSampleSize = private_nh.param<float>("downSampleSize", 0.05f);
            map_tf = private_nh.param<std::string>("map_tf", "map");
            base_lidar_tf = private_nh.param<std::string>("base_lidar_tf", "velodyne");
            imu_odom_data.set_capacity(512);
            velosity_x=velosity_y=velosity_yaw=0;


            //init_pose
            curr_pose.setIdentity(4,4);
            curr_pose.block(0,0,3,3) = euler2rot(0,0,init_yaw);
            curr_pose(0,3) =init_x;
            curr_pose(1,3) =init_y;
            curr_pose(2,3) =-trim_low;
            curr_pose_stamp =ros::Time(0.001);
            //ndt_omp
            pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
            ndt->setTransformationEpsilon(0.01);
            ndt->setResolution(1.0);
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            registration = ndt;
            //registration-icp
//            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr regis (new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
//            registration = regis;
//            registration->setMaximumIterations(icpMaximumIterations);
//            registration->setRANSACOutlierRejectionThreshold(icpRANSACOutlierRejectionThreshold);
//            registration->setMaxCorrespondenceDistance(icpRANSACOutlierRejectionThreshold*100);
//            registration->setTransformationEpsilon(icpTransformationEpsilon);
//        registration.set
            /**sub and pub**/
//            odom_pub = nh.advertise<nav_msgs::Odometry>(map_tf,50);
            odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",50);
            get_pmsg_pub = nh.advertise<nav_msgs::Odometry>("/stamp",5);
            points_suber = nh.subscribe("/velodyne_points",1,&RealTime_Localization::points_callback,this);
            localmap_suber =  mt_nh.subscribe("/localmap",1,&RealTime_Localization::localmap_callback,this);
            imu_odom_suber = mt_nh.subscribe("/akermanslamlidarnode",512,&RealTime_Localization::imu_callback,this);
            curr_pointcloud_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/registered_pointCloud",5);
////temp add
            Regis_input_odom = nh.advertise<nav_msgs::Odometry>("/regis_in_odom",50);
            /**utility param**/
            downSampler.setLeafSize(downSampleSize,downSampleSize,downSampleSize);

            NODELET_INFO("realTime_localization_nodelet initial completed");
        }

    private:

        void imu_callback(const nav_msgs::OdometryConstPtr& odom_msg){
            std::lock_guard<std::mutex> odom_lock(imu_odom_data_mutex);
            imu_odom_data.push_back(odom_msg);
        }

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
            auto flat_cloud  = trimInputCloud(filtered_cloud,nearPointThreshold,farPointThreshold, trim_low, trim_high);

            //select closest imu odom pose (bad performance)
            Matrix4f odom_pose;
            if (imu_odom_data.size()!=0)
            {
                for (boost::circular_buffer<nav_msgs::OdometryConstPtr>::const_iterator i = imu_odom_data.end() - 1;
                     i != imu_odom_data.begin(); i--) {
                    nav_msgs::Odometry odometry = **i;
                    NODELET_INFO("pass a odom");
                    if (odometry.header.stamp < points_msg->header.stamp) {
                        Quaternionf q(odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
                                      odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z);
                        odom_pose.block(0, 0, 3, 3) = quat2rot(q);
                        odom_pose(0, 3) = odometry.pose.pose.position.x;
                        odom_pose(1, 3) = odometry.pose.pose.position.y;
                        odom_pose(2, 3) = curr_pose(2,3);
                        odometry.pose.pose.position.z=curr_pose(2,3);
                        Regis_input_odom.publish(odometry);
                        curr_pose = odom_pose;
                        break;
                    }
                }

            }


            //pc register  (with predict pose by former movement)
//            cout<<curr_pose<<endl;
            Matrix4f predict_pose = predict(curr_pose,curr_pose_stamp,points_msg->header.stamp);
//            cout<<"predict_pose:"<<endl<<predict_pose<<endl;
            nav_msgs::Odometry odometry  = rotm2odometry(predict_pose,points_msg->header.stamp,map_tf,base_lidar_tf);
            Regis_input_odom.publish(odometry);
            Matrix4f transform = pc_register(flat_cloud, localmap_cloud, curr_pose);//predict_pose
            update_velocity(curr_pose,transform,curr_pose_stamp,points_msg->header.stamp);
//            cout<<"velosity_x:\t"<<velosity_x<<endl;
//            cout<<"velosity_y:\t"<<velosity_y<<endl;
//            cout<<"velosity_yaw:\t"<<velosity_yaw<<endl;
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
            nav_msgs::Odometry odom = rotm2odometry(transform,points_msg->header.stamp,map_tf,base_lidar_tf);
            odom_pub.publish(odom);

            //publish cloud raw (optional)
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coloredCloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::RGB>::Ptr color (new pcl::PointCloud<pcl::RGB>());
            //raw cloud
//            pcl::transformPointCloud(*curr_cloud,*transformedCloud,transform);
            //process cloud
            pcl::transformPointCloud(*flat_cloud,*transformedCloud,transform);
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
                                                                const float disNear, const float disFar, const float low, const float high) const {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTrimmed (new pcl::PointCloud<pcl::PointXYZ>());
            cloudTrimmed->reserve(cloud->size());
            std::copy_if(cloud->begin(),cloud->end(),std::back_inserter(cloudTrimmed->points),
                         [&](const pcl::PointXYZ& p){
                             double d = p.getVector3fMap().norm();
                             return d > disNear && d < disFar && p.z > low && p.z <high;
                         }
            );
            cloudTrimmed->width = cloudTrimmed->points.size();
            cloudTrimmed->height = 1;
            cloudTrimmed->header = cloud->header;
            return cloudTrimmed;
        }



        void update_velocity(const Eigen::Matrix4f& original_pose , const Eigen::Matrix4f& registered_pose,const ros::Time& oringal_time, const ros::Time& curr_time){
            Eigen::Vector3f original_oritetion = rot2euler(original_pose.block(0,0,3,3));
            Eigen::Vector3f registered_oritetion = rot2euler(registered_pose.block(0,0,3,3));
            double time_colleasep = (curr_time.toSec()-oringal_time.toSec());
//            cout<<"yaw: \t"<<original_oritetion(2)<<endl;
//            cout<<"yaw d:\t"<<registered_oritetion(2)-original_oritetion(2)<<endl;
            velosity_yaw = (registered_oritetion(2)-original_oritetion(2))/time_colleasep;
            velosity_x = (registered_pose(0,3)-original_pose(0,3))/time_colleasep;
            velosity_y = (registered_pose(1,3)-original_pose(1,3))/time_colleasep;
        }

        Eigen::Matrix4f predict(const Eigen::Matrix4f& original_pose, const ros::Time& oringal_time, const ros::Time& curr_time){
            double time_colleasep = (curr_time.toSec()-oringal_time.toSec());
            Eigen::Matrix4f future_pose;
            Eigen::Vector3f original_oritetion = rot2euler(original_pose.block(0,0,3,3));
            // should correct oritenan
//            future_pose.block(0,0,3,3) =  euler2rot(0,0,original_oritetion(2)+velosity_yaw*time_colleasep);
            future_pose.block(0,0,3,3) =  original_pose.block(0,0,3,3);
            future_pose(0,3) = original_pose(0,3)+velosity_x*time_colleasep;
            future_pose(1,3) = original_pose(1,3)+velosity_y*time_colleasep;
            future_pose(2,3) = original_pose(2,3);
            return future_pose;
        }

        Eigen::Matrix4f pc_register(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& curr_cloud,const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& local_map,const Eigen::Matrix4f& initial_matrix){
            //registration
            registration->setInputSource(curr_cloud);
            registration->setInputTarget(local_map);
            pcl::PointCloud<pcl::PointXYZ> result_cloud ;
            //registration start
            clock_t start = clock();
            registration->align(result_cloud,initial_matrix);//,curr_pose
            cout<<"registration->getFitnessScore()\t"<<registration->getFitnessScore()<<endl;
            clock_t end = clock();
            NODELET_INFO("registration regis time = %f seconds",(double)(end  - start) / CLOCKS_PER_SEC);
//            NODELET_INFO("fitness score = %f ",registration.getFitnessScore());
//            NODELET_INFO(" ");
            return registration->getFinalTransformation();
//        Vector3f euler = rot2euler(transform.block(0,0,3,3));
//        NODELET_INFO("x = %f, y = %f, theta = %f ",transform(0,3),transform(1,3),euler(2));
        }

    private:

        //ros node handle
        ros::NodeHandle nh;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;
        // para
        int curr_step;
        float trim_low;
        float trim_high;
        float farPointThreshold;
        float nearPointThreshold;
        Eigen::Matrix4f curr_pose;
        ros::Time curr_pose_stamp;
        double velosity_x;
        double velosity_y;
        double velosity_yaw;
        // suber and puber
        ros::Publisher curr_pointcloud_pub;
        ros::Subscriber odom_suber;
        ros::Subscriber points_suber;
        ros::Subscriber localmap_suber;
        ros::Subscriber imu_odom_suber;
        ros::Publisher odom_pub;
        ros::Publisher get_pmsg_pub;
        ros::Publisher Regis_input_odom;
        ros::Timer timer;
        tf::TransformBroadcaster transformBroadcaster;
        //tf
        string map_tf;
        string base_lidar_tf;

        // clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr localmap_cloud;

        // utility
        pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;
        pcl::VoxelGrid<pcl::PointXYZ> downSampler;
        std::mutex curr_pose_mutex;
        std::mutex imu_odom_data_mutex;
        // buffer
        boost::circular_buffer<nav_msgs::OdometryConstPtr> imu_odom_data;
        // time log
        ros::Duration full_time;
        double_t average_regis_time;
        int regis_num;


    };


}
PLUGINLIB_EXPORT_CLASS(rt_localization_ns::RealTime_Localization, nodelet::Nodelet)

//                NODELET_INFO("sec:%i,nsec:%i",points_msg->header.stamp.sec,points_msg->header.stamp.nsec);
//                NODELET_INFO("sec:%i,nsec:%i",curr_pose_stamp.sec,curr_pose_stamp.nsec);

//        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;

//            cout<<"odom_pose: "<<endl<<odom_pose<<endl;
//            cout<<"calculate_pose: "<<endl<<transform<<endl;


