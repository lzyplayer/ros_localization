//林肯车，地库场景

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
struct State
{
    Eigen::Matrix<double, 2, 1> pos;//位置
    double heading;//朝向
    double v;
    double header;
}start, current, change, out;

ros::Publisher odom_output_pub;
queue<State> M;

double delta;
double time_;
bool inite = false;
bool slam_inite = false;//初始化
void imucallback(const sensor_msgs::ImuConstPtr& input)
{
    double timestamp = input->header.stamp.toSec();
    if(!inite)////////////////////////////////////解决第一次的问题
    {
        inite = true;
        start.pos << 0, 0;
        start.heading = 0.0;
        start.v = 0.0;
        start.header = timestamp;
        current.pos << 0, 0;
        current.heading = 0.0;
        current.v = 0.0;
        current.header = timestamp;
        change.pos << 0.0, 0.0;
        change.heading = 0.0;
        change.v = 0.0;
        change.header = 0.0;
        out.pos << 0, 0;
        out.heading = 0.0;
        out.v = 0.0;
        out.header = timestamp;
        return;
    }
    if(!slam_inite)
    {
        time_ = timestamp;
        return;
    }
    //四元数输入－>YAW－>处理－>四元数输出
    ///imu与车体校准//////////////////////////////////////////////////////////////////////////////////////////////////////
        double A_ = input->linear_acceleration.x;
        double W_ = input->angular_velocity.z;
        double T_ = current.heading;//单位×××××××××××××××××××××××××××××××××××××××××××rad
        double X_ = current.pos(0);
        double Y_ = current.pos(1);
        double V_ = current.v;
    {
        const double LIMITW = 0.01;

        double txv, tyv, t2xa, t2ya, Ttxv_w, Ttyv_w;
        delta = timestamp - time_;
        double delta_1 = delta;
        double delta_2 = delta * delta;
        double delta_3 = delta * delta * delta; 
        if(abs(W_) > LIMITW)
        {
            double ypwT = T_ + W_*delta;
            double w_1  = 1.0/ W_;
            double w_2  = 1.0/(W_ * W_);

            txv  =   w_1 * (sin(ypwT) - sin(T_));
            tyv  = - w_1 * (cos(ypwT) - cos(T_));
            t2xa = 2 * (  delta * w_1 * sin(ypwT) + w_2 * (cos(ypwT)-cos(T_)));
            t2ya = 2 * (- delta * w_1 * cos(ypwT) + w_2 * (sin(ypwT)-sin(T_)));
            Ttxv_w = (delta * txv - t2xa)/W_;
            Ttyv_w = (delta * tyv - t2ya)/W_;
        }
        else
        {
            txv =               delta_1 * cos(T_);
            tyv =               delta_1 * sin(T_);
            t2xa =              delta_2 * cos(T_);
            t2ya =              delta_2 * sin(T_);
            Ttxv_w = ( 1/6.0) * delta_3 * sin(T_);
            Ttyv_w = (-1/6.0) * delta_3 * cos(T_);
        }
        change.pos << V_ * txv + A_ * t2xa / 2.0, V_ * tyv + A_ * t2ya / 2.0;//单次变化
        change.heading = W_ * delta;
        change.v = A_ * delta;
        current.pos += change.pos;
        current.heading += change.heading;
        current.v += change.v;
        current.header = timestamp;
    }
    out.pos += change.pos;
    out.heading += change.heading;
    out.v += change.v;
    out.header = timestamp;
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

        odom_output.twist.twist.linear.x = out.v * cos(out.heading);
        odom_output.twist.twist.linear.y = out.v * sin(out.heading);
        odom_output.twist.twist.linear.z = 0.0;
        odom_output.twist.twist.angular.x = 0.0;
        odom_output.twist.twist.angular.y = 0.0;
        odom_output.twist.twist.angular.z = W_;

        odom_output_pub.publish(odom_output);
    }
}

void slamsigcallback(const sensor_msgs::ImuConstPtr& input)
{
    //记录积分段的起点位姿
    M.push(current);
}

void slamcallback(const nav_msgs::OdometryConstPtr& input)//slam获取后再启动算法？？？？？？？？？？？？？？？？？？？？？？？？
{//时间戳是获取帧的时间戳
    slam_inite = true;
    //判断乱序
    if(!M.empty())
    {
        start=M.front();//取出
        M.pop();//删除
        std::cout << "start" << start.header <<std::endl;
        std::cout << "input" << input->header.stamp.toSec() <<std::endl;
        while(start.header != input->header.stamp.toSec();)
        {
            start=M.front();//取出
            M.pop();//丢弃
        }
    }
    out.pos << input->pose.pose.position.x + current.pos(0) - start.pos(0), input->pose.pose.position.y + current.pos(1) - start.pos(1);//切

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
    out.heading = yaw + current.heading - start.heading;
}
   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slamlocalization");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    odom_output_pub = node.advertise<nav_msgs::Odometry>("/yuyaoodometry", 10000);//pub出估计的结果
    
    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 100, imucallback);
    ros::Subscriber SLAMLIDAR_sub = node.subscribe("/odom", 10000, slamcallback);
    ros::Subscriber SLAMSIG_sub = node.subscribe("/stamp", 10000, slamsigcallback);

    ros::spin();//循环
}