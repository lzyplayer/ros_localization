/**
 * 融合slam位置、slam姿态、加速度、角速度、激光里程计
 */
#include <string>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iterator>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <queue>
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

#include "ukf_core/measurements/position_measurement.hpp"
#include "ukf_core/measurements/heading_measurement.hpp"
#include "ukf_core/measurements/angularvelocity_measurement.hpp"
#include "ukf_core/measurements/acceleration_measurement.hpp"
#include "ukf_core/measurements/velocity_measurement.hpp"

#include "ukf_core/models/smodel_ukf2d.hpp"

#include "ukf_core/unscented_kalman_filter.hpp"

using namespace std;
using namespace chrono;//计时用的
using Eigen::MatrixXd;
using Eigen::VectorXd;

AngularvelocityMeasurement g_imu_angularvelocity_measurement;
AccelerationMeasurement g_imu_acceleration_measurement;
PositionMeasurement g_slam_position_measurement;
HeadingMeasurement g_slam_heading_measurement;
VelocityMeasurement g_lidar_velocity_measurement;
VelocityMeasurement g_slam_velocity_measurement;
AngularvelocityMeasurement g_lidar_angularvelocity_measurement;

bool g_slam_measurement_changed;
bool g_lidar_measurement_changed;;
bool inite_g_slam_measurement = false;
bool inite_g_lidar_measurement = false;

//传感器本身给定的误差水平,也许可以改成自适应
//SLAM信号弱情况下，要提高其不准确度，根据covar可以输出体现
double cov_imuacc, cov_imuang, cov_slamyaw, cov_slampos, cov_slamvel, cov_lidarang, cov_lidarvel;
double ppos, pv, pyaw, pa, pw;
bool flag_linearacc;//去向心加速度
bool flag_cali;//IMU系是否进行算法标定
bool flag_slamv;//是否使用SLAM速度

struct Zpre_lidar
{
    double vx, vy, w;//获取时的预测,速度，角速度，位置，朝向
    double stamp;//时间，为了验证
};
queue<Zpre_lidar> M_lidar;
Zpre_lidar n_lidar;

nav_msgs::Odometry odom;//封装
ros::Publisher odom_s_pub;

UnscentedKalmanFilter<SModel> g_filter_s;
decltype(g_filter_s)::Model::FilterVector outputstate_s_;//实时输出

int N = 0;
double rostimebegin, rostimeend;
int output_count_ = 0;
double z_=0;

double delta_;
double t2_;
double timestamp_old;
Eigen::Matrix<double, 2, 1> measurement1_lidar;//v
Eigen::Matrix<double, 1, 1> measurement2_lidar;//w

bool inite = false;
void update(double timestamp, ros::Time stamp);

boost::mutex inputmutex;//////////

Eigen::IOFormat eigen_csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, "," , "," , "" , "" , "", "");

void SlamCallback(const nav_msgs::OdometryConstPtr& input)
{
    double timestamp = input->header.stamp.toSec();
    {
        double a = input->pose.pose.position.x;
        if(!std::isnan(a))//判断是否是nan
        {
            //pos
            Eigen::Matrix<double, 2, 1> measurement;
            measurement << input->pose.pose.position.x, input->pose.pose.position.y;
            Eigen::Matrix<double, 2, 2> covariance;
            covariance = Eigen::MatrixXd::Identity(2,2) * cov_slampos;
            z_ = input->pose.pose.position.z;
            // double cov = input->pose.covariance[0]/input->pose.covariance[1] * 0.1;//比例设定，适配于整个系统
            // covariance << cov, 0,
            // 0,cov;
            g_slam_position_measurement.setMeasurement(measurement, covariance, timestamp);

            //yaw
            geometry_msgs::Quaternion orientation;
            orientation.x = input->pose.pose.orientation.x;
            orientation.y = input->pose.pose.orientation.y;
            orientation.z = input->pose.pose.orientation.z;
            orientation.w = input->pose.pose.orientation.w;
            tf2::Quaternion orientation_quat;
            tf2::fromMsg(orientation, orientation_quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3 orTmp(orientation_quat);
            orTmp.getRPY(roll, pitch, yaw);            
            Eigen::Matrix<double, 1, 1> measurement1;
            measurement1 << yaw;
            Eigen::Matrix<double, 1, 1> covariance1;
            covariance1 << cov_slamyaw;
            g_slam_heading_measurement.setMeasurement(measurement1, covariance1, timestamp);

            //v
            if(flag_slamv)
            {
                measurement << input->twist.twist.linear.x, input->twist.twist.linear.y;
                covariance = Eigen::MatrixXd::Identity(2,2) * cov_slamvel;
                g_slam_velocity_measurement.setMeasurement(measurement, covariance, timestamp);
            }

            if(N == 0)  
            {
                g_filter_s.init(timestamp, input->pose.pose.position.x, input->pose.pose.position.y, yaw);
                inite = true;
                inite_g_slam_measurement = true;
                N += 1;
            }
            g_slam_measurement_changed = true;
        }
    }
}
void LidarsigCallback(const nav_msgs::OdometryConstPtr& input)//记录状态信息
{
    t2_=input->header.stamp.toSec();//bag时间戳,存储
    cout << "lidar_signal" << endl;

    int total_length_s = 0;
    total_length_s += decltype(g_filter_s)::getLengthForMeasurement<AccelerationMeasurement>();
    total_length_s += decltype(g_filter_s)::getLengthForMeasurement<AngularvelocityMeasurement>();
    if(g_slam_measurement_changed)
    {
        total_length_s += decltype(g_filter_s)::getLengthForMeasurement<PositionMeasurement>();
        total_length_s += decltype(g_filter_s)::getLengthForMeasurement<HeadingMeasurement>();
    } 
    if(g_lidar_measurement_changed) 
    {
        total_length_s += decltype(g_filter_s)::getLengthForMeasurement<VelocityMeasurement>();
        total_length_s += decltype(g_filter_s)::getLengthForMeasurement<AngularvelocityMeasurement>();
    }
    bool f = true;
    g_filter_s.beginAddMeasurement(t2_, total_length_s, f);

    Zpre_lidar Zs;
    Zs.vx = g_filter_s.x_pre(2);//prediction;
    Zs.vy = g_filter_s.x_pre(3);
    Zs.w = g_filter_s.x_pre(7);
    Zs.stamp = t2_;//点对第一帧时间戳
    M_lidar.push(Zs);
    cout << "**M_lidar" << M_lidar.size() << "**" << endl;
}
//保证ｓｋ对齐机制，不要漏帧错帧　　附带当时的时间戳
void LidarCallback(const nav_msgs::OdometryConstPtr& input)// 处理演化过程
{
    double timestamp = input->header.stamp.toSec() + input->twist.twist.linear.x;//处理后的时间
    timestamp_old = input->header.stamp.toSec();//点云获取时间
    if(!inite)//模型初始化
    {
        // g_filter_s.init(timestamp);
        return;
        inite = true;
    }
    else
    {
        geometry_msgs::Quaternion orientation;
        orientation.x = input->pose.pose.orientation.x;
        orientation.y = input->pose.pose.orientation.y;
        orientation.z = input->pose.pose.orientation.z;
        orientation.w = input->pose.pose.orientation.w;
        tf2::Quaternion orientation_quat;
        tf2::fromMsg(orientation, orientation_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3 orTmp(orientation_quat);
        orTmp.getRPY(roll, pitch, yaw);
    
        /*--------------------------------------------------------------------*\
        \*--------------------------------------------------------------------*/
        if(!M_lidar.empty())
        {
            Zpre_lidar n;//用来记录读出来的东西； 
            n=M_lidar.front();
            double t1 = n.stamp;
            M_lidar.pop();
            Zpre_lidar n_new;
            n_new = M_lidar.front();
            double delta = n_new.stamp - t1;
            cout<<"*%%M_lidar*"<< M_lidar.size() <<"**"<<endl;
            std::cout << std::setprecision(15) << "LIDAR后一个:" << n_new.stamp << std::endl;
            std::cout << std::setprecision(15) << "LIDAR delta:" << delta << std::endl;
            std::cout << std::setprecision(15) << "LIDAR栈顶" << n.stamp <<std::endl;
            std::cout << std::setprecision(15) << "LIDAR输入" << timestamp_old <<std::endl;
            while(n_new.stamp != timestamp_old)//因为第一次数据到，里程计是不输出的/////////////////////////////////////////////////////////////////
            {
                std::cout << std::setprecision(15) << "LIDAR不对齐了ｎ（＊≧▽≦＊）ｎ" <<std::endl;
                std::cout << std::setprecision(15) << "LIDAR栈顶" << n.stamp <<std::endl;
                std::cout << std::setprecision(15) << "LIDAR输入" << timestamp_old <<std::endl;
                if(n_new.stamp > timestamp_old)
                {
                    return;
                }
                else
                {
                    n = M_lidar.front();
                    t1 = n.stamp;
                    M_lidar.pop();
                    delta = M_lidar.front().stamp - t1;
                }
            }

            double a = input->pose.pose.position.x;
            if(!std::isnan(a))//判断是否是nan
            {
                inite_g_lidar_measurement = true;

                Eigen::Matrix<double, 2, 1> measurement;
                double X_ = input->pose.pose.position.x/delta;
                double Y_ = input->pose.pose.position.y/delta;
                cout << "lidarvx vy" << X_ << endl << Y_ << endl;
                double vx;
                double vy;
                vx = X_*cos(outputstate_s_(4)) - Y_*sin(outputstate_s_(4));
                vy = X_*sin(outputstate_s_(4)) + Y_*cos(outputstate_s_(4));///IMU系
                measurement << vx - n.vx, vy - n.vy;
                Eigen::Matrix<double, 2, 2> covariance;
                covariance = Eigen::MatrixXd::Identity(2,2) * cov_lidarvel;
                g_lidar_velocity_measurement.setMeasurement(measurement, covariance, timestamp);
                measurement1_lidar = measurement;

                Eigen::Matrix<double, 1, 1> measurement1;
                measurement1 << yaw/delta - n.w;
                Eigen::Matrix<double, 1, 1> covariance1;
                covariance1 << cov_lidarang;
                g_lidar_angularvelocity_measurement.setMeasurement(measurement1, covariance1, timestamp);
                measurement2_lidar = measurement1;
                
                g_lidar_measurement_changed = true;
            }
        }
    }
}
void imuCallback(const sensor_msgs::ImuConstPtr& input)
{
    rostimebegin = ros::Time::now().toSec();

    bool imuinite = true;
    double timestamp = input->header.stamp.toSec();
    ros::Time stamp = input->header.stamp;
    {
        if(!inite)
        {
            imuinite = false;
        }
        else
        {
            //w
            Eigen::Matrix<double, 1, 1> angularvelocitymeasurement;
            angularvelocitymeasurement << input->angular_velocity.z;
            Eigen::Matrix<double, 1, 1> angularvelocitycovariance;
            angularvelocitycovariance << cov_imuang;
            g_imu_angularvelocity_measurement.setMeasurement(angularvelocitymeasurement, angularvelocitycovariance, timestamp);

            //a
            Eigen::Matrix<double, 2, 1> accelerationmeasurement;
            accelerationmeasurement << input->linear_acceleration.x, input->linear_acceleration.y;

            if(flag_linearacc)
            {
                Eigen::Matrix<double, 2, 1> V;
                V << outputstate_s_(3), -outputstate_s_(2);
                accelerationmeasurement += angularvelocitymeasurement(0) * V;
            }
            if (flag_cali)  
            {
                Eigen::Matrix<double, 2, 2> C2;
                C2 << 0, -1,
                      -1, 0;
                accelerationmeasurement = C2 * accelerationmeasurement;
            }

            Eigen::Matrix<double, 2, 2> accelerationcovariance;
            accelerationcovariance = Eigen::MatrixXd::Identity(2,2) * cov_imuacc;
            g_imu_acceleration_measurement.setMeasurement(accelerationmeasurement, accelerationcovariance, timestamp);
        }
    }
    if(imuinite && inite_g_slam_measurement)
    {
        update(timestamp, stamp);
    }
    rostimeend = ros::Time::now().toSec();
}

void update(double timestamp, ros::Time stamp)//来什么量测，用什么来更新
{
    bool use_slam = g_slam_measurement_changed;
    bool use_lidar = g_lidar_measurement_changed;

    boost::mutex::scoped_lock lock(inputmutex);
    {
        int total_length_s = 0;//先统计这次更新用多少维
        total_length_s += decltype(g_filter_s)::getLengthForMeasurement<AccelerationMeasurement>();
        total_length_s += decltype(g_filter_s)::getLengthForMeasurement<AngularvelocityMeasurement>();
        if(use_slam) 
        {
            total_length_s += decltype(g_filter_s)::getLengthForMeasurement<PositionMeasurement>();
            total_length_s += decltype(g_filter_s)::getLengthForMeasurement<HeadingMeasurement>();
            if(flag_slamv) total_length_s += decltype(g_filter_s)::getLengthForMeasurement<VelocityMeasurement>();
        }
        if(use_lidar) 
        {
            total_length_s += decltype(g_filter_s)::getLengthForMeasurement<VelocityMeasurement>();
            total_length_s += decltype(g_filter_s)::getLengthForMeasurement<AngularvelocityMeasurement>();
        }

        bool f = false;
        g_filter_s.beginAddMeasurement(timestamp, total_length_s, f);
        
        g_filter_s.AddMeasurement(g_imu_angularvelocity_measurement);
        g_filter_s.AddMeasurement(g_imu_acceleration_measurement);
        if(use_slam) 
        {
            g_filter_s.AddMeasurement(g_slam_position_measurement);
            g_filter_s.AddMeasurement(g_slam_heading_measurement);
            if(flag_slamv) g_filter_s.AddMeasurement(g_slam_velocity_measurement);
        }
        if(use_lidar)//对量测进行二次推演，改变
        {            
            Eigen::Matrix<double, 2, 1> measurement1;
            Eigen::Matrix<double, 1, 1> measurement2;
            //v
            measurement1 << g_filter_s.x_pre(2), g_filter_s.x_pre(3);
            g_lidar_velocity_measurement.changeMeasurement(measurement1);
            g_filter_s.AddMeasurement(g_lidar_velocity_measurement);
            measurement1_lidar += measurement1;
            std::cout<<"LIDARvel:"<< measurement1_lidar <<std::endl;
            //w
            measurement2 << g_filter_s.x_pre(7);
            g_lidar_angularvelocity_measurement.changeMeasurement(measurement2);
            g_filter_s.AddMeasurement(g_lidar_angularvelocity_measurement);
            measurement2_lidar += measurement2;
            std::cout<<"LIDARangular:"<< measurement2_lidar <<std::endl;
        }
    
        g_filter_s.midAddMeasurement();
        
        g_filter_s.AddMeasurement2(g_imu_angularvelocity_measurement);
        g_filter_s.AddMeasurement2(g_imu_acceleration_measurement);
        if(use_slam) 
        {
            g_filter_s.AddMeasurement2(g_slam_position_measurement);
            if(flag_slamv) g_filter_s.AddMeasurement2(g_slam_velocity_measurement);
            g_filter_s.AddMeasurement2(g_slam_heading_measurement);
        }
        if(use_lidar)
        {
            g_filter_s.AddMeasurement2(g_lidar_velocity_measurement);
            g_filter_s.AddMeasurement2(g_lidar_angularvelocity_measurement);
        }
    }

    if(use_lidar) g_lidar_measurement_changed = false;
    if(use_slam) g_slam_measurement_changed = false;

    decltype(g_filter_s)::Model::FilterVector outputstate_s;
    decltype(g_filter_s)::Model::FilterMatrix outputcovariance_s;
    double outputtime_s;
    
    g_filter_s.endAddMeasurement(outputstate_s, outputcovariance_s, outputtime_s);//最终估计结果
    cout << "outputstate_s" << outputstate_s <<endl;
    outputstate_s_ = outputstate_s;

    nav_msgs::Odometry odom_s;//封装
    odom_s.header.stamp = odom.header.stamp;
    odom_s.header.frame_id = "odom";
    odom_s.child_frame_id = "base_foot";
    odom_s.pose.pose.position.x = outputstate_s(0);
    odom_s.pose.pose.position.y = outputstate_s(1);
    odom_s.pose.pose.position.z = z_;

    tf2::Quaternion quat_s;
    quat_s.setRPY(0, 0, outputstate_s(4));
    odom_s.pose.pose.orientation.x = quat_s.x();
    odom_s.pose.pose.orientation.y = quat_s.y();
    odom_s.pose.pose.orientation.z = quat_s.z();
    odom_s.pose.pose.orientation.w = quat_s.w();

    odom_s.twist.twist.linear.x = outputstate_s(2);//IMU系
    odom_s.twist.twist.linear.y = outputstate_s(3);
    odom_s.twist.twist.linear.z = 0.0;
    odom_s.twist.twist.angular.x = 0.0;
    odom_s.twist.twist.angular.y = 0.0;
    odom_s.twist.twist.angular.z = outputstate_s(7) - outputstate_s(10);
    
    odom_s_pub.publish(odom_s);

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
    ros::init(argc, argv, "sukf2dslam");
    ros::NodeHandle node;

    cov_slampos = node.param<double>("cov_slampos", 0.01);
    cov_slamyaw = node.param<double>("cov_slamyaw", 0.01);
    cov_slamvel = node.param<double>("cov_slamvel", 0.1);
    cov_imuacc = node.param<double>("cov_imuacc", 0.1);
    cov_imuang = node.param<double>("cov_imuang", 0.1);
    cov_lidarang = node.param<double>("cov_lidarang", 0.1);
    cov_lidarvel = node.param<double>("cov_lidarvel", 0.1);

    flag_linearacc = node.param<bool>("flag_linearacc", false);
    flag_slamv = node.param<bool>("flag_slamv", false);
    flag_cali = node.param<bool>("flag_cali", false);

    ppos = node.param<double>("ppos", 0.05);
    pv = node.param<double>("pv", 0.05);
    pyaw = node.param<double>("pyaw", 0.02);
    pa = node.param<double>("pa", 1);
    pw = node.param<double>("pw", 0.05);

    SModelParameter para(ppos, pv, pyaw, pa, pw);
    g_filter_s.setModelParameter(para);

    odom_s_pub = node.advertise<nav_msgs::Odometry>("/sukf2dslamrealodometry", 10000);//pub出估计的结果

    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 10, imuCallback);//200hz
    ros::Subscriber SLAMPOS_sub = node.subscribe("/lp_odom", 10000, SlamCallback);//10hz
    // ros::Subscriber SLAMPOS_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/filteredodometry", 10000, SlamCallback);//10hz
    // ros::Subscriber SLAMPOS_sub = node.subscribe("/odometry", 10000, SlamCallback);//10hz
    ros::Subscriber LIDAR_sub = node.subscribe("/lidar_odom", 10000, LidarCallback);//10hz
    ros::Subscriber LIDARSIG_sub = node.subscribe("/lidar_stamp", 10000, LidarsigCallback);//10hz
    ros::spin();//循环
}

