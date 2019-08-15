/**
 * 融合位置、姿态、加速度、角速度
 */
#include <string>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iterator>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>/////////////////////tf2
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Transform.h>
#include <message_filters/subscriber.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "ukf_core/measurements3d/position_measurement.hpp"
#include "ukf_core/measurements3d/heading_measurement.hpp"
#include "ukf_core/measurements3d/angularvelocity_measurement.hpp"
#include "ukf_core/measurements3d/acceleration_measurement.hpp"

#include "ukf_core/models/smodel_ukf3d.hpp"

#include "ukf_core/unscented_kalman_filter.hpp"

using namespace std;
using namespace chrono;//计时用的
using Eigen::MatrixXd;
using Eigen::VectorXd;

AngularvelocityMeasurement g_imu_angularvelocity_measurement;
AccelerationMeasurement g_imu_acceleration_measurement;
PositionMeasurement g_slam_position_measurement;
HeadingMeasurement g_slam_heading_measurement;

bool g_slam_position_measurement_changed;
bool g_slam_heading_measurement_changed;

bool inite_g_slam_position_measurement = false;
bool inite_g_slam_heading_measurement = false;

double rostimebegin;
double rostimeend;
double rostimepub;
double oldpub = 0;
double deltapub = 0;
double afterpub = 0;
double pubtime = 0;
double z_=0;
double angular_z_=0;

int output_count_ = 0;
int N = 0;
//传感器本身给定的误差水平,也许可以改成自适应
//SLAM信号弱情况下，要提高其不准确度，根据covar可以输出体现
Eigen::MatrixXd covar_imuacc = Eigen::MatrixXd::Identity(3,3) * 0.01;
Eigen::MatrixXd covar_imuang = Eigen::MatrixXd::Identity(3,3) * 0.01;
Eigen::MatrixXd covar_slamyaw = Eigen::MatrixXd::Identity(4,4) * 0.1;

SModelParameter para;
UnscentedKalmanFilter<SModel> g_filter_s(para);


bool inite = false;
void update(double timestamp);
nav_msgs::Odometry odom;//封装
ros::Publisher rawodoms_pub ,odom_s_pub;

tf2::Quaternion quat_s_;
Eigen::Matrix<double, 3, 1> t_s_;
double w_=0.0;

boost::mutex inputmutex;//////////

Eigen::IOFormat eigen_csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, "," , "," , "" , "" , "", "");

void slamCallback(const nav_msgs::OdometryConstPtr& input)// 三维位置、三维姿态
{
    //模拟
    {
        int seq=input->header.seq;
        if(N%3 != 0)
        {
            N += 1;
            //std::cout << "断掉" << std::endl;
            return;
        }
    }
    
    double timestamp = input->header.stamp.toSec();

    if(!inite)//模型初始化
    {
        g_filter_s.init(timestamp);
        inite = true;
    }
    else
    {
           //POS
        double a = input->pose.pose.position.x;
        if(!std::isnan(a))//判断是否是nan
        {
            inite_g_slam_position_measurement = true;

            Eigen::Matrix<double, 3, 1> measurement;
            measurement << input->pose.pose.position.x - 313896.082284, input->pose.pose.position.y - 3791429.257284, input->pose.pose.position.z;
            Eigen::Matrix<double, 3, 3> covariance;
            covariance << input->pose.covariance[0] * input->pose.covariance[0]*100, 0, 0,
            0,input->pose.covariance[1] * input->pose.covariance[1]*100, 0,
            0, 0, input->pose.covariance[2] * input->pose.covariance[2]*100;
            
            g_slam_position_measurement.setMeasurement(measurement, covariance, timestamp);
            g_slam_position_measurement_changed = true;

            //std::cout << "SLAM方差=" << covariance << std::endl;
            //std::cout<<"SLAMpos:"<< measurement <<std::endl;
        }

        
        //姿态
        a=input->pose.pose.orientation.w;
        if(!std::isnan(a))
        {//四元数
            inite_g_slam_heading_measurement = true;//////////////////////////////////////////////////////

            // geometry_msgs::Quaternion orientation;
            Eigen::Quaterniond orientation(input->pose.pose.orientation.w, input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z);
            orientation.normalize();

            Eigen::Matrix<double, 4, 1> measurement;
            measurement << orientation.w(), orientation.x(), orientation.y(), orientation.z();
            Eigen::Matrix<double, 4, 4> covariance;
            covariance << covar_slamyaw;

            g_slam_heading_measurement.setMeasurement(measurement, covariance, timestamp);
            g_slam_heading_measurement_changed = true;
        }
    }
}

void imuCallback(const sensor_msgs::ImuConstPtr& input)
{
    rostimebegin = ros::Time::now().toSec();///////////////////////////////////////////////////////////////////////////////////////////////时间对齐

    bool imuinite = true;
    double timestamp = input->header.stamp.toSec();
    {
        if(!inite)
        {
            imuinite = false;
            g_filter_s.init(timestamp);
            inite = true;
        }
        else
        {
            Eigen::Matrix<double, 3, 1> angularvelocitymeasurement;
            angularvelocitymeasurement << input->angular_velocity.x, input->angular_velocity.y, input->angular_velocity.z;
            Eigen::Matrix<double, 3, 3> angularvelocitycovariance;
            angularvelocitycovariance << covar_imuang;

            g_imu_angularvelocity_measurement.setMeasurement(angularvelocitymeasurement, angularvelocitycovariance, timestamp);


            Eigen::Matrix<double, 3, 1> accelerationmeasurement;
            accelerationmeasurement << input->linear_acceleration.x, input->linear_acceleration.y, input->linear_acceleration.z;
            // Eigen::Matrix<double, 3, 3> W;
            // W << 0.0329983, -0.999447, 0.00401395,
            //     -0.999455,  -0.0330029, -0.00110399,
            //      0.00123585, -0.00397533, -0.999991;
            // accelerationmeasurement = W * accelerationmeasurement;     
            
            Eigen::Matrix<double, 3, 3> accelerationcovariance;
            accelerationcovariance << covar_imuacc;

            g_imu_acceleration_measurement.setMeasurement(accelerationmeasurement, accelerationcovariance, timestamp);
        }
    }
    if(imuinite && inite_g_slam_position_measurement && inite_g_slam_heading_measurement)
    {
        odom.header.stamp=input->header.stamp;
        update(timestamp);

    }
    rostimeend = ros::Time::now().toSec();
    //std::cout<< std::setprecision(15)<<"rostimeend:"<<rostimeend<<std::endl;
}

void update(double timestamp)//来什么量测，用什么来更新
{
    bool use_slampos = g_slam_position_measurement_changed;
    bool use_slamyaw = g_slam_heading_measurement_changed;

    boost::mutex::scoped_lock lock(inputmutex);////////////////////////////////////////////////////////////////////////////////////////////////////

    {
        int total_length_s = 0;//先统计这次更新用多少维
        total_length_s += decltype(g_filter_s)::getLengthForMeasurement<AccelerationMeasurement>();
        total_length_s += decltype(g_filter_s)::getLengthForMeasurement<AngularvelocityMeasurement>();
        if(use_slampos) total_length_s += decltype(g_filter_s)::getLengthForMeasurement<PositionMeasurement>();
        if(use_slamyaw) total_length_s += decltype(g_filter_s)::getLengthForMeasurement<HeadingMeasurement>();

        g_filter_s.beginAddMeasurement(timestamp, total_length_s);

        g_filter_s.AddMeasurement(g_imu_angularvelocity_measurement);
        g_filter_s.AddMeasurement(g_imu_acceleration_measurement);
        if(use_slampos) g_filter_s.AddMeasurement(g_slam_position_measurement);
        if(use_slamyaw) g_filter_s.AddMeasurement(g_slam_heading_measurement);
        
        g_filter_s.midAddMeasurement();
        
        g_filter_s.AddMeasurement2(g_imu_angularvelocity_measurement);
        g_filter_s.AddMeasurement2(g_imu_acceleration_measurement);
        if(use_slampos) g_filter_s.AddMeasurement2(g_slam_position_measurement);
        if(use_slamyaw) g_filter_s.AddMeasurement2(g_slam_heading_measurement);
    }

    if(use_slampos) g_slam_position_measurement_changed = false;
    if(use_slamyaw) g_slam_heading_measurement_changed = false;

    decltype(g_filter_s)::Model::FilterVector outputstate_s;
    decltype(g_filter_s)::Model::FilterMatrix outputcovariance_s;
    double outputtime_s;
    
    g_filter_s.endAddMeasurement(outputstate_s, outputcovariance_s, outputtime_s);//最终估计结果

    PositionMeasurement::MeasurementVector position_s;
    SensorMeasurementConverter::getMeasurementPub<PositionMeasurement, decltype(g_filter_s)::Model>(g_filter_s.model_parameter_, outputstate_s, position_s);

    HeadingMeasurement::MeasurementVector heading_s;
    SensorMeasurementConverter::getMeasurementPub<HeadingMeasurement, decltype(g_filter_s)::Model>(g_filter_s.model_parameter_, outputstate_s, heading_s);

    AngularvelocityMeasurement::MeasurementVector angularvelocity_s;
    SensorMeasurementConverter::getMeasurementPub<AngularvelocityMeasurement, decltype(g_filter_s)::Model>(g_filter_s.model_parameter_, outputstate_s, angularvelocity_s);

    AccelerationMeasurement::MeasurementVector acceleration_s;
    SensorMeasurementConverter::getMeasurementPub<AccelerationMeasurement, decltype(g_filter_s)::Model>(g_filter_s.model_parameter_, outputstate_s, acceleration_s);
    
    std::cout << "估计" << outputstate_s << std::endl;
    //std::cout << "估计pos=" << position_s << std::endl;

    nav_msgs::Odometry odom_s;//封装
    odom_s.header.stamp = odom.header.stamp;
    odom_s.header.frame_id = "odom";
    odom_s.child_frame_id = "base_foot";

    odom_s.pose.pose.position.x = position_s(0) + 313896.082284;
    odom_s.pose.pose.position.y = position_s(1) + 3791429.257284;
    odom_s.pose.pose.position.z = position_s(2);

    odom_s.pose.pose.orientation.w = heading_s(0);
    odom_s.pose.pose.orientation.x = heading_s(1);
    odom_s.pose.pose.orientation.y = heading_s(2);
    odom_s.pose.pose.orientation.z = heading_s(3);

    odom_s.twist.twist.linear.x = outputstate_s(3);
    odom_s.twist.twist.linear.y = outputstate_s(4);
    odom_s.twist.twist.linear.z = outputstate_s(5);
    odom_s.twist.twist.angular.x = angularvelocity_s(0);
    odom_s.twist.twist.angular.y = angularvelocity_s(1);
    odom_s.twist.twist.angular.z = angularvelocity_s(2);

    
    odom_s_pub.publish(odom_s);
    //std::cout<< std::setprecision(15)<<"rostimepub:"<<rostimepub<<std::endl;
    //std::cout<< std::setprecision(15)<<"deltapub:"<<deltapub<<std::endl;
    //std::cout<< std::setprecision(15)<<"pubtime:"<<pubtime<<std::endl;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped OdomBasefootprintTransMsg_;
    OdomBasefootprintTransMsg_.header.stamp = ros::Time::now();
    OdomBasefootprintTransMsg_.header.frame_id = "odom";
    OdomBasefootprintTransMsg_.child_frame_id = "base_foot";

    OdomBasefootprintTransMsg_.transform.translation.x = odom_s.pose.pose.position.x;
    OdomBasefootprintTransMsg_.transform.translation.y = odom_s.pose.pose.position.y;
    OdomBasefootprintTransMsg_.transform.translation.z = 0.0;
    OdomBasefootprintTransMsg_.transform.rotation = odom_s.pose.pose.orientation;
    br.sendTransform(OdomBasefootprintTransMsg_);

    output_count_ = output_count_ + 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizationsukf");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    odom_s_pub = node.advertise<nav_msgs::Odometry>("/sukfodometry", 10000);//pub出估计的结果
    rawodoms_pub = node.advertise<nav_msgs::Odometry>("/rawodoms", 10000);///预处理x y值
   
    ros::Subscriber SLAMPOS_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/fixtoodometry", 10000, slamCallback);//10hz
    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 10, imuCallback);//200hz
    ros::spin();//循环
}

