/**
 * 在ＳＬＡＭ基础上积分加速度和角速度，提供更高频的信息
 * 考虑自由运动
 * 队列处理
**/
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
#include <queue>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
//#include <yuyao/Zpre.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Transform.h>
#include <message_filters/subscriber.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace chrono;//计时用的
//**************************************************************************************//
//用于计算积分结果
struct TIME
{
    double header;
    Eigen::Matrix<double, 2, 1> v;
}start, current;

struct State
{
    Eigen::Matrix<double, 2, 1> pos;//绝对位置
    double heading;//朝向
    Eigen::Matrix<double, 2, 1> v;//IMU系
    double header;
}out;

struct IMU
{
    Eigen::Matrix<double, 2, 1> a;//绝对位置
    double w;//朝向
    double header;
    double delta;
}record,imu,imu1;

ros::Publisher odom_output_pub;
queue<TIME> M;
queue<IMU> sensor;

double delta;
double time_;
double timestamp;
bool inite = false;
bool slam_inite = false;//初始化
void updatenew(double ax, double ay, double wz, double delta);
void imucallback(const sensor_msgs::ImuConstPtr& input)
{
    timestamp = input->header.stamp.toSec();
    if(!inite)////////////////////////////////////解决第一次的问题
    {
        inite = true;

        start.header = timestamp;
        current.header = timestamp;
        out.pos << 0, 0;
        out.heading = 0.0;
        out.v << 0, 0;
        out.header = timestamp;
        time_ = timestamp;
        return;
    }
    double delta = timestamp - time_;
    time_ = timestamp;
    //存储snesor信息
    record.w = input->angular_velocity.z;
    record.header = timestamp;
    record.a(0) = input->linear_acceleration.x;
    record.a(1) = input->linear_acceleration.y;
    record.delta = delta;
    sensor.push(record);

    if(!slam_inite)
    {
        return;
    }
    //四元数输入－>YAW－>处理－>四元数输出
    ///imu与车体校准//////////////////////////////////////////////////////////////////////////////////////////////////////
    
    updatenew(record.a(0), record.a(1), record.w, delta);//当前积分

    current.header = timestamp;
    current.v = out.v;


    {//发布topic
        nav_msgs::Odometry odom_output;
        odom_output.header.stamp = input->header.stamp;
        odom_output.header.frame_id = "odom";
        odom_output.child_frame_id = "base_foot";

        odom_output.pose.pose.position.x = out.pos(0);
        odom_output.pose.pose.position.y = out.pos(1);
        odom_output.pose.pose.position.z = 0;

        tf2::Quaternion quat_bicycle_;
        quat_bicycle_.setRPY(0, 0, out.heading);
        odom_output.pose.pose.orientation.x = quat_bicycle_.x();
        odom_output.pose.pose.orientation.y = quat_bicycle_.y();
        odom_output.pose.pose.orientation.z = quat_bicycle_.z();
        odom_output.pose.pose.orientation.w = quat_bicycle_.w();

        odom_output.twist.twist.linear.x = out.v(0) * cos(out.heading) + out.v(1) * sin(out.heading);//IMU系
        odom_output.twist.twist.linear.y = -out.v(0) * sin(out.heading) + out.v(1) * cos(out.heading);
        //odom_output.twist.twist.linear.x = out.v(0);
        //odom_output.twist.twist.linear.y = out.v(1);
        odom_output.twist.twist.linear.z = 0.0;
        odom_output.twist.twist.angular.x = 0.0;
        odom_output.twist.twist.angular.y = 0.0;
        odom_output.twist.twist.angular.z = 0.0;

        odom_output_pub.publish(odom_output);
    }
}

void slamsigCallback(const nav_msgs::OdometryConstPtr& input)
{
    //记录积分段的起点speed
    current.header = input->header.stamp.toSec();
    M.push(current);
}

void slamCallback(const nav_msgs::OdometryConstPtr& input)//slam获取后再启动算法？？？？？？？？？？？？？？？？？？？？？？？？
{
    double in = input->header.stamp.toSec();////时间戳是获取帧的时间戳
    slam_inite = true;
    //判断乱序
    out.pos << input->pose.pose.position.x, input->pose.pose.position.y;//切

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
    out.heading = yaw;

    if(!M.empty() && !sensor.empty())
    {
        start=M.front();//取出,记录帧的时间戳
        M.pop();
        std::cout << std::setprecision(15) << "start" << start.header <<std::endl;
        std::cout << std::setprecision(15) <<"input" << in <<std::endl;

        while(start.header != in)
        {
            start = M.front();
            
            imu=sensor.front();//取出

            while(start.header > imu.header)
            {
                sensor.pop();
                imu = sensor.front();
            }
            M.pop();
        }
        cout << "jieshu" <<endl;
    }
    else
    {
        std::cout << "error" << std::endl;
    }
    ////updatenew
    out.v = start.v;
    int size = sensor.size();
    for(int i = 0; i++; i < size)
    {
        imu = sensor.front();
        cout << "kaishi" << endl;
        updatenew(imu.a(0),imu.a(1),imu.w,imu.delta);//重新积分
        if(imu.header <= start.header)
        {
            sensor.pop();
        }
        else
        {
            imu1 = sensor.front();
            sensor.pop();
            sensor.push(imu1);
        }
    }
}

void updatenew(double ax, double ay, double wz, double delta)
{
    double W_ = wz;
    double T_ = out.heading;//单位×××××××××××××××××××××××××××××××××××××××××××rad
    double X_ = out.pos(0);
    double Y_ = out.pos(1);
    double VX_ = out.v(0);
    double VY_ = out.v(1);
    Eigen::Matrix<double, 2, 1> AF_;
    Eigen::Matrix<double, 2, 1> A_;
    Eigen::Matrix<double, 2, 1> V_;
    V_ << VX_, VY_;
    AF_ << ax, ay;

    double delta_1 = delta;
    double delta_2 = delta * delta;

    Eigen::Matrix<double, 2, 2> Cbe;
    Cbe << cos(T_), -sin(T_),
            sin(T_), cos(T_);
    Eigen::Matrix<double, 2, 2> Ceb;
    Ceb << -sin(T_), cos(T_),
            cos(T_), sin(T_);

    //A_ = Cbe * (AF_ + W_ * Ceb * V_);
    A_ = Cbe * AF_;
    double AX_ = A_(0);
    double AY_ = A_(1);
    double PX_  = X_  + VX_ * delta_1 + AX_ * delta_2 / 2;
    double PY_  = Y_  + VY_ * delta_1 + AY_ * delta_2 / 2;
    double PVX_ = VX_ + AX_ * delta_1;
    double PVY_ = VY_ + AY_ * delta_1;
    double PT_ = T_ + W_ * delta;

    out.pos << PX_, PY_;
    out.heading = PT_;
    out.v << PVX_, PVY_;
    out.header = timestamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slamlocalization");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    odom_output_pub = node.advertise<nav_msgs::Odometry>("/imu_odometry", 10000);//pub出估计的结果
    
    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 100, imucallback);
    ros::Subscriber SLAMLIDAR_sub = node.subscribe("/odom", 10000, slamCallback);
    ros::Subscriber SLAMSIG_sub = node.subscribe("/stamp", 10000, slamsigCallback);

    ros::spin();//循环
}