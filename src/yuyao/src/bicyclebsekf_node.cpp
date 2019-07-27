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

#include "ekf_core/measurements/position_measurement.hpp"
#include "ekf_core/measurements/velocity_measurement.hpp"
#include "ekf_core/measurements/heading_measurement.hpp"
#include "ekf_core/measurements/angularvelocity_measurement.hpp"
#include "ekf_core/measurements/acceleration_measurement.hpp"
#include "ekf_core/measurements/steeringwheel_measurement.hpp"

#include "ekf_core/models/bicycle.hpp"

#include "ekf_core/extended_kalman_filter.hpp"

using namespace std;
using namespace chrono;//计时用的

PositionMeasurement g_gps_position_measurement;//在measurements里定义的类
HeadingMeasurement g_gps_heading_measurement;
AngularvelocityMeasurement g_imu_angularvelocity_measurement;
AccelerationMeasurement g_imu_acceleration_measurement;
SteeringwheelangleMeasurement g_sw_angle_measurement;//方向盘转角
VelocityMeasurement g_vehicle_velocity_measurement;//轮速
//slammeasurement
//PitchMeasurement g_imu_pitch_measurement;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool g_gps_position_measurement_changed;
bool g_gps_heading_measurement_changed;
bool g_sw_angle_measurement_changed;
bool g_vehicle_velocity_measurement_changed;
//bool g_lidar_measurement_changed;
//bool use_imupitch;

bool inite_g_gps_position_measurement = false;//////////////////////////////////////////////////////////只有IMU没有？？？？？？？？？？
bool inite_g_gps_heading_measurement = false;
bool inite_g_sw_angle_measurement = false;
bool inite_g_vehicle_velocity_measurement = false;
//bool inite_g_lidar_measurement = false;

ExtendedKalmanFilter<BICYCLEModel> g_filter_bicycle;

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
//GPS信号弱情况下，要提高其不准确度，根据covar可以输出体现
double covar_gpsyaw=0.04;//双天线朝向结果，会有NAN情况
double covar_swang=0.04;//不太准确
double covar_vehvel=0.01;
double covar_imuacc=0.01;
double covar_imuang=0.01;

double L=14.8;//方向盘转角比车轮转角/////////////////////////////////////////////////////////////////////////////
double w_limit=0.05;

//#define DEVEHVEL//转弯时去掉轮速信息
//#define DEBUG
//#define DEBUG1
#ifdef DEBUG
double last_w = 0;
#endif

bool inite = false;
void update(double timestamp);
nav_msgs::Odometry odom;//封装
ros::Publisher rawodombicycle_pub ,odom_bicycle_pub;


// ofstream mycout_x("/home/midou/pxx.txt");
// ofstream mycout_y("/home/midou/pyy.txt");
// ofstream mycout_s("/home/midou/pss.txt");
// ofstream mycout_v("/home/midou/pvv.txt");
// ofstream mycout_w("/home/midou/pww.txt");
// ofstream mycout_a("/home/midou/paa.txt");

boost::mutex inputmutex;/////////////////////////////////////////////////////////////////////////////////////////////////、、、、、、、、、、

Eigen::IOFormat eigen_csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, "," , "," , "" , "" , "", "");

void gpsCallback(const nav_msgs::OdometryConstPtr& input)// POS VEL YAW
{
    {/////////////////////////////////////////////////////////////gpstest//////////////////////////////////////
        //GPS良好情况下人为去掉GPS信号输入
        /*
        {
            N = N+1;  
            if(N>240)
            {
                std::cout << "断掉GPS，算法还准吗" << std::endl;
                return;
            }
        }
        */

        //boost::mutex::scoped_lock lock(inputmutex);


        //查看停车时段，是否继续往前走,序列号是“siyuan”bag的停车部分
        /*
        {
            int seq=input->header.seq;
            if(seq>1970&&seq<2170)
            {
                std::cout << "停车时段，继续往前就错咯" << std::endl;
                return;
            }
        }
        */


        //入地库之前断掉GPS信号输入,序列号是“diku”bag的下地库部分
        
        {
            int seq=input->header.seq;
            if(seq>62900)
            {
                std::cout << "断GPS" << std::endl;
                return;
            }
        }
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double timestamp = input->header.stamp.toSec();

    if(!inite)//模型初始化
    {
        g_filter_bicycle.init(timestamp);
        inite = true;
    }
    else
    {
            //GPS POS
        double a = input->pose.pose.position.x;
        if(!std::isnan(a))//判断是否是nan
        {
            inite_g_gps_position_measurement = true;//GPS测量

            Eigen::Matrix<double, 2, 1> measurement;
            measurement << input->pose.pose.position.x - 313896.082284, input->pose.pose.position.y - 3791429.257284;
            Eigen::Matrix<double, 2, 2> covariance;
            covariance << input->pose.covariance[0] * input->pose.covariance[0]*100, 0,
            0,input->pose.covariance[1] * input->pose.covariance[1]*100;/////////////////////////////////////////////////含义？？？？
            g_gps_position_measurement.setMeasurement(measurement, covariance, timestamp);
            z_=input->pose.pose.position.z;

            g_gps_position_measurement_changed = true;
            std::cout<<"GPSpos:"<< measurement <<std::endl;
        }

        
        //GPS YAW 使用的是原始双天线朝向，不完全输入，存在NAN
        a=input->pose.pose.orientation.w;
        if(!std::isnan(a))
        {//将四元数转换为欧拉角
            inite_g_gps_heading_measurement = true;//////////////////////////////////////////////////////

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

            Eigen::Matrix<double, 1, 1> measurement;
            measurement << yaw;
            Eigen::Matrix<double, 1, 1> covariance;
            covariance << covar_gpsyaw;
            g_gps_heading_measurement.setMeasurement(measurement, covariance, timestamp);

            g_gps_heading_measurement_changed = true;
            std::cout<<"GPSyaw:"<< yaw <<std::endl;
        }    
    }
    //结构
    //header(seq、frame_id、stamp) pose(pose.position、pose.orientation、conv) twist(twist.angular、twist.linear、conv)
        nav_msgs::Odometry rawodom;
        rawodom.header.stamp = ros::Time::now();
        rawodom.header.frame_id = "odom";  ////////
        rawodom.child_frame_id = "base_foot";////////////、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、
        rawodom.pose.pose.position.x = input->pose.pose.position.x - 313896.082284;
        rawodom.pose.pose.position.y = input->pose.pose.position.y - 3791429.257284;
        rawodom.pose.pose.position.z = 0;
        rawodom.pose.pose.orientation = input->pose.pose.orientation;
        rawodom.twist = input->twist;

        rawodombicycle_pub.publish(rawodom);///
}

// ////****ADD****////
void swhangCallback(const dbw_mkz_msgs::SteeringReportConstPtr& input)
{
        //boost::mutex::scoped_lock lock(inputmutex);
    double timestamp = input->header.stamp.toSec();

    if(!inite)
    {
        g_filter_bicycle.init(timestamp);
        inite = true;
    }
    else
    {
        double a = input->steering_wheel_angle;
        if(!std::isnan(a))
        {
            inite_g_sw_angle_measurement=true;

            Eigen::Matrix<double, 1, 1> measurement;
            measurement << input->steering_wheel_angle/L;
            Eigen::Matrix<double, 1, 1> swanglecovariance;
            swanglecovariance << covar_swang;
            g_sw_angle_measurement.setMeasurement(measurement, swanglecovariance, timestamp);//写入成员变量

            g_sw_angle_measurement_changed = true;
            std::cout<<"VEHangle:"<< measurement <<std::endl;
        }

        a = input->speed;//车轴方向速度
        if(!std::isnan(a))
        {
            inite_g_vehicle_velocity_measurement=true;

            Eigen::Matrix<double, 1, 1> measurement;
            measurement << input->speed;
            Eigen::Matrix<double, 1, 1> vehvelcovariance;
            vehvelcovariance << covar_vehvel;
            if(input->speed == 0)
            {
                vehvelcovariance << 0.08;
            }
            g_vehicle_velocity_measurement.setMeasurement(measurement, vehvelcovariance, timestamp);//写入成员变量

            g_vehicle_velocity_measurement_changed = true;
            std::cout<<"VEHvel:"<< measurement <<std::endl;
        }
    }

}



void imuCallback(const sensor_msgs::ImuConstPtr& input)
{
    rostimebegin = ros::Time::now().toSec();///////////////////////////////////////////////////////////////////////////////////////////////时间对齐
    //std::cout<< std::setprecision(15)<<"rostimebegin:"<<rostimebegin<<std::endl;
    auto start = system_clock::now();//计时

    bool imuinite = true;
    double timestamp = input->header.stamp.toSec();
    {
        //boost::mutex::scoped_lock lock(inputmutex);
        if(!inite)
        {
            imuinite = false;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            g_filter_bicycle.init(timestamp);
            inite = true;
        }
        else
        {
            Eigen::Matrix<double, 1, 1> angularvelocitymeasurement;
            angularvelocitymeasurement << input->angular_velocity.z;
            Eigen::Matrix<double, 1, 1> angularvelocitycovariance;
            angularvelocitycovariance << covar_imuang;
            g_imu_angularvelocity_measurement.setMeasurement(angularvelocitymeasurement, angularvelocitycovariance, timestamp);
            angular_z_=input -> angular_velocity.z;

            Eigen::Matrix<double, 1, 1> accelerationmeasurement;
            accelerationmeasurement << input->linear_acceleration.x;
            Eigen::Matrix<double, 1, 1> accelerationcovariance;
            accelerationcovariance << covar_imuacc;
            g_imu_acceleration_measurement.setMeasurement(accelerationmeasurement, accelerationcovariance, timestamp);
        }
    }
    //std::cout<<"imuinite"<<imuinite<<inite_g_gps_position_measurement<<inite_g_gps_velocity_measurement<<inite_g_gps_heading_measurement<<std::endl;
    if(imuinite&&inite_g_gps_position_measurement)
    //if(imuinite)//执行条件
    {
        odom.header.stamp=input->header.stamp;
        update(timestamp);

    }

    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    //cout<< std::setprecision(15) <<  "花费了spend" << double(duration.count()) * microseconds::period::num / microseconds::period::den << "ssss秒" << endl;

    rostimeend = ros::Time::now().toSec();
    //std::cout<< std::setprecision(15)<<"rostimeend:"<<rostimeend<<std::endl;
}

void update(double timestamp)//来什么量测，用什么来更新
{

    bool use_gpspos = g_gps_position_measurement_changed;
    bool use_gpsyaw = g_gps_heading_measurement_changed;
    bool use_vehvel = g_vehicle_velocity_measurement_changed;
    bool use_swangle = g_sw_angle_measurement_changed;
#ifdef DEVEHVEL
    if(abs(angular_z_)>w_limit)
    {
        use_vehvel=false;
    }
#endif

#ifdef DEBUG
    double last_w = 0;
    std::cout << "USAGE: IMU | ";
    if(use_gpspos) std::cout<< "GPSPOS" << " | ";
    if(use_gpsyaw) std::cout<< "GPSYAW" << " | ";
    if(use_gpsvel) std::cout<< "GPSVEL" << " | ";
    if(use_vehvel) std::cout<< "VEHVEL" << " | ";
    if(use_swangle) std::cout<< "SWANGLE" << " | ";
    std::cout << std::endl;
#endif
    {
        boost::mutex::scoped_lock lock(inputmutex);////////////////////////////////////////////////////////////////////////////////////////////////////
 //bicycle
        {
            int total_length_bicycle = 0;//先统计这次更新用多少维
            total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AccelerationMeasurement>();
            total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
            if(use_gpspos) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<PositionMeasurement>();
            if(use_gpsyaw) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<HeadingMeasurement>();
            if(use_swangle) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<SteeringwheelangleMeasurement>();
            if(use_vehvel) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<VelocityMeasurement>();
        //std::cout<<"totallength:" << total_length;
            g_filter_bicycle.beginAddMeasurement(timestamp, total_length_bicycle);
            g_filter_bicycle.AddMeasurement(g_imu_angularvelocity_measurement);
            g_filter_bicycle.AddMeasurement(g_imu_acceleration_measurement);
            if(use_gpspos) g_filter_bicycle.AddMeasurement(g_gps_position_measurement);
            if(use_gpsyaw) g_filter_bicycle.AddMeasurement(g_gps_heading_measurement);
            if(use_swangle) g_filter_bicycle.AddMeasurement(g_sw_angle_measurement);
            if(use_vehvel) g_filter_bicycle.AddMeasurement(g_vehicle_velocity_measurement);
        }

        if(use_gpspos) g_gps_position_measurement_changed = false;
        if(use_gpsyaw) g_gps_heading_measurement_changed = false;
        if(use_swangle) g_sw_angle_measurement_changed = false;
        if(use_vehvel) g_vehicle_velocity_measurement_changed = false;
    }
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////covarience????????????????

    decltype(g_filter_bicycle)::Model::FilterVector outputstate_bicycle;
    decltype(g_filter_bicycle)::Model::FilterMatrix outputcovariance_bicycle;
    double outputtime_bicycle;

    g_filter_bicycle.endAddMeasurement(outputstate_bicycle, outputcovariance_bicycle, outputtime_bicycle);

    // mycout_x << outputcovariance_bicycle(0,0) << endl;
    // mycout_y << outputcovariance_bicycle(1,1) << endl;
    // mycout_s << outputcovariance_bicycle(2,2) << endl;
    // mycout_v << outputcovariance_bicycle(3,3) << endl;
    // mycout_w << outputcovariance_bicycle(4,4) << endl;
    // mycout_a << outputcovariance_bicycle(5,5) << endl;

    // std::cout << "P = "<< outputcovariance_bicycle <<std::endl;
    // std::cout << "Q = "<< g_filter_bicycle.Q_ <<std::endl;
    // std::cout << "W = "<< g_filter_bicycle.W_ <<std::endl;

    PositionMeasurement::MeasurementVector position_bicycle;
    SensorMeasurementConverter::getMeasurementPrediction<PositionMeasurement, decltype(g_filter_bicycle)::Model>(g_filter_bicycle.model_parameter_, outputstate_bicycle, position_bicycle);

    HeadingMeasurement::MeasurementVector heading_bicycle;
    SensorMeasurementConverter::getMeasurementPrediction<HeadingMeasurement, decltype(g_filter_bicycle)::Model>(g_filter_bicycle.model_parameter_, outputstate_bicycle, heading_bicycle);

    VelocityMeasurement::MeasurementVector velocity_bicycle;
    SensorMeasurementConverter::getMeasurementPrediction<VelocityMeasurement, decltype(g_filter_bicycle)::Model>(g_filter_bicycle.model_parameter_, outputstate_bicycle, velocity_bicycle);

    AngularvelocityMeasurement::MeasurementVector angularvelocity_bicycle;
    SensorMeasurementConverter::getMeasurementPrediction<AngularvelocityMeasurement, decltype(g_filter_bicycle)::Model>(g_filter_bicycle.model_parameter_, outputstate_bicycle, angularvelocity_bicycle);

    AccelerationMeasurement::MeasurementVector acceleration_bicycle;
    SensorMeasurementConverter::getMeasurementPrediction<AccelerationMeasurement, decltype(g_filter_bicycle)::Model>(g_filter_bicycle.model_parameter_, outputstate_bicycle, acceleration_bicycle);

    nav_msgs::Odometry odom_bicycle;//封装
    odom_bicycle.header.stamp = odom.header.stamp;
    odom_bicycle.header.frame_id = "odom";
    odom_bicycle.child_frame_id = "base_foot";

    odom_bicycle.pose.pose.position.x = position_bicycle(0)+313896.082284 + 1.02* cos(outputstate_bicycle(2));
    odom_bicycle.pose.pose.position.y = position_bicycle(1)+3791429.257284 + + 1.02* sin(outputstate_bicycle(2));
    odom_bicycle.pose.pose.position.z = z_;

    tf2::Quaternion quat_bicycle;
    quat_bicycle.setRPY(0, 0, heading_bicycle(0));
    odom_bicycle.pose.pose.orientation.x = quat_bicycle.x();
    odom_bicycle.pose.pose.orientation.y = quat_bicycle.y();
    odom_bicycle.pose.pose.orientation.z = quat_bicycle.z();
    odom_bicycle.pose.pose.orientation.w = quat_bicycle.w();


    odom_bicycle.twist.twist.linear.x = velocity_bicycle(0) * cos(heading_bicycle(0));
    odom_bicycle.twist.twist.linear.y = velocity_bicycle(0) * sin(heading_bicycle(0));
    odom_bicycle.twist.twist.linear.z = 0.0;
    odom_bicycle.twist.twist.angular.x = 0.0;
    odom_bicycle.twist.twist.angular.y = 0.0;
    odom_bicycle.twist.twist.angular.z = angularvelocity_bicycle(0);

    
    odom_bicycle_pub.publish(odom_bicycle);
    //std::cout<< std::setprecision(15)<<"rostimepub:"<<rostimepub<<std::endl;
    //std::cout<< std::setprecision(15)<<"deltapub:"<<deltapub<<std::endl;
    //std::cout<< std::setprecision(15)<<"pubtime:"<<pubtime<<std::endl;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped OdomBasefootprintTransMsg_;
    OdomBasefootprintTransMsg_.header.stamp = ros::Time::now();
    OdomBasefootprintTransMsg_.header.frame_id = "odom";
    OdomBasefootprintTransMsg_.child_frame_id = "base_foot";

    OdomBasefootprintTransMsg_.transform.translation.x = odom_bicycle.pose.pose.position.x;
    OdomBasefootprintTransMsg_.transform.translation.y = odom_bicycle.pose.pose.position.y;
    OdomBasefootprintTransMsg_.transform.translation.z = 0.0;
    OdomBasefootprintTransMsg_.transform.rotation = odom_bicycle.pose.pose.orientation;
    br.sendTransform(OdomBasefootprintTransMsg_);
#ifdef DEBUG
    cout << "  XY : " << outputstate(0) << ", \t" << outputstate(1) << ", \t";
    if(use_gpspos) cout << "REF: " << g_gps_position_measurement.measurement_(0) << ", \t" << g_gps_position_measurement.measurement_(1);
    cout << endl;

    cout << "  TW : " << outputstate(2) << ", \t" << outputstate(4) << ", \t";
    cout << "REF: ";
    if(use_gpsyaw) cout << g_gps_heading_measurement.measurement_(0) <<  ", \t";
    else cout << "---, \t";
    cout << g_imu_angularvelocity_measurement.measurement_(0) << endl;

    cout << "  VA : " << outputstate(3) << ", \t" << outputstate(5) << ", \t";
    cout << "REF: ";
    if(use_gpsvel) cout << g_gps_velocity_measurement.measurement_(0) <<  ", \t";
    else cout << "---, \t";
    cout << g_imu_acceleration_measurement.measurement_(0) << endl;

    //if(abs(outputstate(4)) > 1) throw;
    if(abs(outputstate(4) - last_w) > 1) throw;
    //if(abs(outputstate(5)) > 0.5) throw;
    if(use_gpspos && abs(outputstate(0) - g_gps_position_measurement.measurement_(0)) > 3) throw;
    if(use_gpspos && abs(outputstate(1) - g_gps_position_measurement.measurement_(1)) > 3) throw;
    last_w = outputstate(4);
#endif

#ifdef DEBUG1
    std::cout << "ERROR W: " << outputstate(4)-g_imu_angularvelocity_measurement.measurement_(0) << "ORIGN: " << g_imu_angularvelocity_measurement.measurement_(0)<< std::endl ;
    std::cout << "ERROR A: " << outputstate(5)-g_imu_acceleration_measurement.measurement_(0) << "ORIGN: " << g_imu_acceleration_measurement.measurement_(0)<< std::endl ;
#endif

    output_count_ = output_count_ + 1;
}

int main(int argc, char **argv)
{
    g_filter_bicycle.setModelParameter(BICYCLEModelParameter(100,100,1.25,1.6)); //sigma2_beta, sigma2_a, l

    ros::init(argc, argv, "localizationbicycle");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    odom_bicycle_pub = node.advertise<nav_msgs::Odometry>("/bicycleodometry", 10000);//pub出估计的结果
    rawodombicycle_pub = node.advertise<nav_msgs::Odometry>("/rawodombicycle", 10000);///预处理x y值

    ros::Subscriber GPSPOS_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/filteredodometry", 10000, gpsCallback);//10hz
    ros::Subscriber GPSPOS_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/fixtoodometry", 10000, gpsCallback);//10hz
    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 10, imuCallback);//200hz
    ros::Subscriber SWHANG_sub = node.subscribe("/vehicle/steering_report", 10000,swhangCallback);//轮速，转角 100hz


    ros::spin();//循环
    // mycout_x.close();
    // mycout_y.close();
    // mycout_s.close();
    // mycout_v.close();
    // mycout_w.close();
    // mycout_a.close();
}//看闭合结果