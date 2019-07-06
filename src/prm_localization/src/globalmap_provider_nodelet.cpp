#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
namespace globalmap_ns {
    using namespace std;

    class GlobalmapProviderNodelet : public nodelet::Nodelet {
    public:
        GlobalmapProviderNodelet() {
        }

        virtual  ~GlobalmapProviderNodelet() {
        }

        void onInit() override {
            /**init**/
            nh = getNodeHandle();
            mt_nh = getMTNodeHandle();
            private_nh = getPrivateNodeHandle();
            curr_pose.reset(new geometry_msgs::Pose2D());
            /**load map and pub once**/ //maybe add voxelgrid down sample
            std::string globalmap_pcd = "/home/vickylzy/WorkSPacesROS/catkin_ws/src/prm_localization/data/yuyao2dl0h4.pcd";//private_nh.param<std::string>("globalmap_pcd", "");
            full_map.reset(new pcl::PointCloud<pcl::PointXY>());
            pcl::io::loadPCDFile(globalmap_pcd, *full_map);
            full_map->header.frame_id = "map";
            globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap",1);
            globalmap_pub.publish(full_map);
            /**trimmer**/
            kdtree.setInputCloud(full_map);
            radius = 40.0f;
            /**sub and pub**/
//            pose_suber = mt_nh.subscribe("/TOPIC_OF_ODOM",1,&GlobalmapProviderNodelet::pose_callback,this);
            localmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/localmap",1);
            timer = nh.createTimer(ros::Duration(0.5),&GlobalmapProviderNodelet::localmap_callback,this);
            NODELET_INFO("globalmap_provider_nodelet initial completed");
        }

    private:
        /**
         * update newest pose
         * @param pose_msg
         */
        void pose_callback(const geometry_msgs::Pose2D& pose_msg){
            *curr_pose = pose_msg;
        }
        /**
         * trim local_map with latest pose
         * @param event
         * !note:publish pointcloud<pcl::PointXYZI>
         * !suggest:publish pointcloud<pcl::PointXY> (given Z I are 0)
         */
        void localmap_callback(const ros::TimerEvent& event){
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            pcl::PointXY searchPoint;
            searchPoint.x = curr_pose->x;
            searchPoint.y = curr_pose->y;
            pcl::PointCloud<pcl::PointXY>::Ptr trimmed_cloud (new pcl::PointCloud<pcl::PointXY>);
            if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {

                trimmed_cloud->width=pointIdxRadiusSearch.size();
                trimmed_cloud->height = 1;
                trimmed_cloud->points.resize(trimmed_cloud->width*trimmed_cloud->height);

                for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                {
                    trimmed_cloud->points[i] = full_map->points[pointIdxRadiusSearch[i]];
//                    trimmed_cloud->points[i].y = full_map->points[pointIdxRadiusSearch[i]].y;
                }
            }
            trimmed_cloud->header.frame_id="map";
            localmap_pub.publish(trimmed_cloud);
            NODELET_INFO(" local map updated");
        }
    public:
        pcl::PointCloud<pcl::PointXY>::Ptr full_map;
    private:
        //ros node handle
        ros::NodeHandle nh;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;
        // suber and puber
        ros::Publisher localmap_pub;
        ros::Publisher globalmap_pub;
        ros::Subscriber pose_suber;
        // parameter
        geometry_msgs::Pose2DPtr curr_pose;
        pcl::KdTreeFLANN< pcl::PointXY > kdtree;
        float radius;
        // ros timer
        ros::Timer timer ;
        // time log
        ros::Duration full_time;


    };

}
PLUGINLIB_EXPORT_CLASS(globalmap_ns::GlobalmapProviderNodelet, nodelet::Nodelet)

//        /**viewer**/
//    pcl::visualization::CloudViewer viewer("window");
//    viewer.showCloud(realtime_localization->full_map);
//    while (!viewer.wasStopped())
//    {
//    }