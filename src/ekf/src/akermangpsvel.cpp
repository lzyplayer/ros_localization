//算法说明
//AKERMAN模型，
//量测使用GPS位置、GPS朝向、IMU、方向盘转角、轮速计
//对齐方式是按IMU，但没有零阶保持

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
#include <dbw_mkz_msgs/WheelSpeedReport.h>
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
#include "ekf_core/measurements/wheel_measurement.hpp"


#include "ekf_core/models/akerman.hpp"
#include "ekf_core/extended_kalman_filter.hpp"

using namespace std;
using namespace chrono;//计时用的

PositionMeasurement g_gps_position_measurement;//在measurements里定义的类
WheelMeasurement g_wheel_velocity_measurement;//4维
HeadingMeasurement g_gps_heading_measurement;
AngularvelocityMeasurement g_imu_angularvelocity_measurement;
AccelerationMeasurement g_imu_acceleration_measurement;
SteeringwheelangleMeasurement g_sw_angle_measurement;//方向盘转角
VelocityMeasurement g_vehicle_velocity_measurement;//车速

//slammeasurement
//PitchMeasurement g_imu_pitch_measurement;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool g_gps_position_measurement_changed;
bool g_gps_heading_measurement_changed;
bool g_sw_angle_measurement_changed;
bool g_wheel_velocity_measurement_changed;
bool g_vehicle_velocity_measurement_changed;

//bool g_lidar_measurement_changed;
//bool use_imupitch;

bool inite_g_gps_position_measurement = false;//////////////////////////////////////////////////////////只有IMU没有？？？？？？？？？？
bool inite_g_gps_heading_measurement = false;
bool inite_g_sw_angle_measurement = false;
bool inite_g_wheel_velocity_measurement = false;
bool inite_g_vehicle_velocity_measurement = false;


ExtendedKalmanFilter<AKERMANModel> g_filter_akerman;

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
double v_=0.02;
double w_=0.01;

//R
//传感器本身给定的误差水平,也许可以改成自适应
//GPS信号弱情况下，要提高其不准确度，根据covar可以输出体现
double covar_gpsyaw=0.04;//双天线朝向结果，会有NAN情况
double covar_vehvel=1;
double covar_swang=0.16;//不太准确
double covar_imuacc=0.01;
double covar_imuang=0.01;

double K=14;//方向盘转角比车轮转角/////////////////////////////////////////////////////////////////////////////
double w_limit=0.05;
double per=1;
int seq_duan = 12100;

//#define DEBUG
//#define DEBUG1
#ifdef DEBUG
double last_w = 0;
#endif

bool inite = false;
void update(double timestamp, ros::Time stamp);

ros::Publisher odom_akermanbs_pub;

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


        //序列号是“sida03”bag的部分
        
        {
            int seq=input->header.seq;
            //if(seq>seq_duan*10) //filter
            if(seq>seq_duan) //fix
            {
                std::cout << "断掉GPS咯" << std::endl;
                return;
            }
        }

        //入地库之前断掉GPS信号输入,序列号是“diku”bag的下地库部分
        // {
        //     int seq=input->header.seq;
        //     if(seq>seq_duan)
        //     {
        //         std::cout << "断GPS" << std::endl;
        //         return;
        //     }
        // }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double timestamp = input->header.stamp.toSec();

    if(!inite)//模型初始化
    {
        g_filter_akerman.init(timestamp);
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

            g_gps_position_measurement_changed = true;
            std::cout<<"GPSpos:"<< measurement <<std::endl;
        }


        //GPS YAW 使用的是原始双天线朝向，不完全输入，存在NAN
        // a=input->pose.pose.orientation.w;
        // if(!std::isnan(a))
        // {//将四元数转换为欧拉角
        //     inite_g_gps_heading_measurement = true;//////////////////////////////////////////////////////

        //     geometry_msgs::Quaternion orientation;
        //     orientation.x = input->pose.pose.orientation.x;
        //     orientation.y = input->pose.pose.orientation.y;
        //     orientation.z = input->pose.pose.orientation.z;
        //     orientation.w = input->pose.pose.orientation.w;
        //     tf2::Quaternion orientation_quat;
        //     tf2::fromMsg(orientation, orientation_quat);
        //     double roll, pitch, yaw;
        //     tf2::Matrix3x3 orTmp(orientation_quat);
        //     orTmp.getRPY(roll, pitch, yaw);
        //     std::cout << "yaw:" << yaw <<endl;
        //     Eigen::Matrix<double, 1, 1> measurement;
        //     measurement << yaw;
        //     Eigen::Matrix<double, 1, 1> covariance;
        //     covariance << covar_gpsyaw;
        //     g_gps_heading_measurement.setMeasurement(measurement, covariance, timestamp);

        //     g_gps_heading_measurement_changed = true;
        //     std::cout<<"GPSyaw:"<< yaw <<std::endl;
        // }
    }
    // else{
    //     std::cout<<"no gps"<<std::endl;
    // }
    //结构
    //header(seq、frame_id、stamp) pose(pose.position、pose.orientation、conv) twist(twist.angular、twist.linear、conv)
}

// ////****ADD****////
void swhangCallback(const dbw_mkz_msgs::SteeringReportConstPtr& input)
{
        //boost::mutex::scoped_lock lock(inputmutex);
    double timestamp = input->header.stamp.toSec();

    if(!inite)
    {
        g_filter_akerman.init(timestamp);
        inite = true;
    }
    else
    {
        double a = input->steering_wheel_angle;
        if(!std::isnan(a))
        {
            inite_g_sw_angle_measurement=true;

            Eigen::Matrix<double, 1, 1> measurement;
            measurement << input->steering_wheel_angle/K;//K是转角比例
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
            vehvelcovariance << covar_vehvel;//////////////////////////////////////////////////////////////////////////////////////////////////////////////////调////
            g_vehicle_velocity_measurement.setMeasurement(measurement, vehvelcovariance, timestamp);//写入成员变量
            g_vehicle_velocity_measurement_changed = true;
            std::cout<<"VEHvel:"<< measurement <<std::endl;
        }
    }
}

void wheelCallback(const dbw_mkz_msgs::WheelSpeedReportConstPtr& input)
{
        //boost::mutex::scoped_lock lock(inputmutex);
    double timestamp = input->header.stamp.toSec();

    if(!inite)
    {
        g_filter_akerman.init(timestamp);
        inite = true;
    }
    else
    {
        double a = input->front_left;
        if(!std::isnan(a))
        {
            if(v_==0 || w_==0)
            {
                return;
            }
            //std::cout<<"使用轮速计" <<std::endl;
            inite_g_wheel_velocity_measurement=true;
            Eigen::Matrix<double, 4, 1> measurement;
            // Eigen::Matrix<double, 4, 1> test;
            // test << 0.5, 0.5, 0.5, 0.5;
            Eigen::Matrix<double, 4, 4> wheelcovariance;
            wheelcovariance << 1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;
            // if(input->front_left< input->front_right && input->rear_left < input->rear_right)//  rad/s
            // {
            //     flag_lr=true;//turn left
            // }
            measurement << input->front_left, 
                           input->front_right, 
                           input->rear_left, 
                           input->rear_right;
            measurement = measurement * per;
            g_wheel_velocity_measurement.setMeasurement(measurement, wheelcovariance, timestamp);//写入成员变量
            g_wheel_velocity_measurement_changed = true;
            //g_wheel_velocity_measurement_changed = false;
            // if(input->front_left < input->front_right && input->rear_left < input->rear_right)//也算是对轮速计结果的一种检验
            // {
            //     measurement << input->front_right, input->front_left, input->rear_right, input->rear_left;
            //     g_wheel_velocity_measurement.setMeasurement(measurement, wheelcovariance, timestamp);//写入成员变量
            //     g_wheel_velocity_measurement_changed = true;
            // }

            std::cout<<"车轮角速度:"<< measurement <<std::endl;
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
    ros::Time stamp=input->header.stamp;
    {
        //boost::mutex::scoped_lock lock(inputmutex);
        if(!inite)
        {
            imuinite = false;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            g_filter_akerman.init(timestamp);
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
            std::cout << "角速度测量结果" << angularvelocitymeasurement << std::endl;
            //安装误差校正
            // Eigen::Matrix<double, 3, 3> W1;
            // Eigen::Matrix<double, 3, 3> W2;
            // Eigen::Matrix<double, 3, 3> W3;
            // Eigen::Matrix<double, 3, 3> W0;
            Eigen::Matrix<double, 3, 3> W;
            // W0 << 0, 1, 0,
            //       -1, 0, 0,
            //       0, 0, 1;
            // W1 << cos(-179.95/57.3), 0, -sin(-179.95/57.3),
            //     0, 1, 0,
            //         sin(-179.95/57.3), 0, cos(-179.95/57.3);
            // W2 << 1, 0, 0,
            //         0, cos(0.23/57.3), sin(0.23/57.3),
            //         0, -sin(0.23/57.3), cos(0.23/57.3);
            // W3 << cos(-178.1221/57.3), sin(-178.1221/57.3), 0,
            //         -sin(-178.1221/57.3), cos(-178.1221/57.3), 0,
            //         0, 0, 1;
            // W = W0*W1*W2*W3;
            W << 0.0329983, -0.999447, 0.00401395,
                -0.999455,  -0.0330029, -0.00110399,
                 0.00123585, -0.00397533, -0.999991;
            // std::cout << "原始x向加速度：" << input->linear_acceleration.x << std::endl;
            // std::cout << "原始y向加速度：" << input->linear_acceleration.y << std::endl;

            //std::cout << "W: " << W << std::endl;
            double acc1;
            acc1 = input->linear_acceleration.x;
            double acc2;
            acc2 = input->linear_acceleration.y;
            double acc3;
            acc3 = input->linear_acceleration.z;
            Eigen::Matrix<double, 3, 1> acc;
            acc(0)=acc1;
            acc(1)=acc2;
            acc(2)=acc3;
            //acc << W*acc;
            Eigen::Matrix<double, 1, 1> accelerationmeasurement;
            accelerationmeasurement << acc(0);
            Eigen::Matrix<double, 1, 1> accelerationcovariance;
            accelerationcovariance << covar_imuacc;
            g_imu_acceleration_measurement.setMeasurement(accelerationmeasurement, accelerationcovariance, timestamp);
            // std::cout << "变换后x向加速度：" << accelerationmeasurement << std::endl;
            // std::cout << "另一种x向加速度：" << acc(0) << std::endl;
        }
    }
    if(imuinite&&inite_g_gps_position_measurement)
    //if(imuinite)//执行条件
    {
         update(timestamp,stamp);
    }

    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    //cout<< std::setprecision(15) <<  "花费了spend" << double(duration.count()) * microseconds::period::num / microseconds::period::den << "ssss秒" << endl;

    rostimeend = ros::Time::now().toSec();
    //std::cout<< std::setprecision(15)<<"rostimeend:"<<rostimeend<<std::endl;
}

void update(double timestamp, ros::Time stamp)//来什么量测，用什么来更新
{
    //std::cout<<"更新开始" <<std::endl;
    bool use_gpspos = g_gps_position_measurement_changed;
    bool use_gpsyaw = g_gps_heading_measurement_changed;
    bool use_swangle = g_sw_angle_measurement_changed;
    bool use_vehvel = g_vehicle_velocity_measurement_changed;
    bool use_whevel = g_wheel_velocity_measurement_changed;

    {
        boost::mutex::scoped_lock lock(inputmutex);////////////////////////////////////////////////////////////////////////////////////////////////////
 //akerman
        {
            int total_length_akerman = 0;//先统计这次更新用多少维
            total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<AccelerationMeasurement>();
            total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<AngularvelocityMeasurement>();
            if(use_gpspos) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<PositionMeasurement>();
            if(use_gpsyaw) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<HeadingMeasurement>();
            if(use_swangle) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<SteeringwheelangleMeasurement>();
            if(use_vehvel) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<VelocityMeasurement>();
            if(use_whevel) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<WheelMeasurement>();
        //std::cout<<"totallength:" << total_length;
            bool f = false;
            g_filter_akerman.beginAddMeasurement(timestamp, total_length_akerman, f);
            g_filter_akerman.AddMeasurement(g_imu_angularvelocity_measurement);
            g_filter_akerman.AddMeasurement(g_imu_acceleration_measurement);
            if(use_gpspos) g_filter_akerman.AddMeasurement(g_gps_position_measurement);
            if(use_gpsyaw) g_filter_akerman.AddMeasurement(g_gps_heading_measurement);
            if(use_swangle) g_filter_akerman.AddMeasurement(g_sw_angle_measurement);
            if(use_whevel) g_filter_akerman.AddMeasurement(g_wheel_velocity_measurement);
            if(use_vehvel) g_filter_akerman.AddMeasurement(g_vehicle_velocity_measurement);
        }

        if(use_gpspos) g_gps_position_measurement_changed = false;
        if(use_gpsyaw) g_gps_heading_measurement_changed = false;
        if(use_swangle) g_sw_angle_measurement_changed = false;
        if(use_whevel) g_wheel_velocity_measurement_changed = false;
        if(use_vehvel) g_vehicle_velocity_measurement_changed = false;

    }
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////covarience????????????????

    decltype(g_filter_akerman)::Model::FilterVector outputstate_akerman;
    decltype(g_filter_akerman)::Model::FilterMatrix outputcovariance_akerman;
    double outputtime_akerman;

    g_filter_akerman.endAddMeasurement(outputstate_akerman, outputcovariance_akerman, outputtime_akerman);

    
    // std::cout << "P = "<< outputcovariance_akerman <<std::endl;
    // std::cout << "Q = "<< g_filter_akerman.Q_ <<std::endl;
    // std::cout << "W = "<< g_filter_akerman.W_ <<std::endl;

    PositionMeasurement::MeasurementVector position_akerman;
    SensorMeasurementConverter::getMeasurementPrediction<PositionMeasurement, decltype(g_filter_akerman)::Model>(g_filter_akerman.model_parameter_, outputstate_akerman, position_akerman);

    HeadingMeasurement::MeasurementVector heading_akerman;
    SensorMeasurementConverter::getMeasurementPrediction<HeadingMeasurement, decltype(g_filter_akerman)::Model>(g_filter_akerman.model_parameter_, outputstate_akerman, heading_akerman);

    VelocityMeasurement::MeasurementVector velocity_akerman;
    SensorMeasurementConverter::getMeasurementPrediction<VelocityMeasurement, decltype(g_filter_akerman)::Model>(g_filter_akerman.model_parameter_, outputstate_akerman, velocity_akerman);

    AngularvelocityMeasurement::MeasurementVector angularvelocity_akerman;
    SensorMeasurementConverter::getMeasurementPrediction<AngularvelocityMeasurement, decltype(g_filter_akerman)::Model>(g_filter_akerman.model_parameter_, outputstate_akerman, angularvelocity_akerman);

    AccelerationMeasurement::MeasurementVector acceleration_akerman;
    SensorMeasurementConverter::getMeasurementPrediction<AccelerationMeasurement, decltype(g_filter_akerman)::Model>(g_filter_akerman.model_parameter_, outputstate_akerman, acceleration_akerman);

    nav_msgs::Odometry odom_akerman;//封装
    //odom_akerman.header.stamp = ros::Time::now();
    odom_akerman.header.stamp = stamp;
    odom_akerman.header.frame_id = "odom";
    odom_akerman.child_frame_id = "base_foot";

    odom_akerman.pose.pose.position.x = position_akerman(0) + 313896.082284;
    odom_akerman.pose.pose.position.y = position_akerman(1) + 3791429.257284;
    odom_akerman.pose.pose.position.z = z_;

    tf2::Quaternion quat_akerman;
    quat_akerman.setRPY(0, 0, heading_akerman(0));
    odom_akerman.pose.pose.orientation.x = quat_akerman.x();
    odom_akerman.pose.pose.orientation.y = quat_akerman.y();
    odom_akerman.pose.pose.orientation.z = quat_akerman.z();
    odom_akerman.pose.pose.orientation.w = quat_akerman.w();
    std::cout<<"速度"<< velocity_akerman(0) <<std::endl;
    //std::cout << "速度"　<< velocity_akerman(0) << std::endl;
    odom_akerman.twist.twist.linear.x = velocity_akerman(0) * cos(heading_akerman(0));
    odom_akerman.twist.twist.linear.y = velocity_akerman(0) * sin(heading_akerman(0));
    odom_akerman.twist.twist.linear.z = 0.0;
    odom_akerman.twist.twist.angular.x = 0.0;
    odom_akerman.twist.twist.angular.y = 0.0;
    odom_akerman.twist.twist.angular.z = angularvelocity_akerman(0);

    v_=outputstate_akerman(3);
    w_=outputstate_akerman(4);
    std::cout<<"角速度"<< angularvelocity_akerman(0) <<std::endl;
    odom_akermanbs_pub.publish(odom_akerman);
    //std::cout<< std::setprecision(15)<<"rostimepub:"<<rostimepub<<std::endl;
    //std::cout<< std::setprecision(15)<<"deltapub:"<<deltapub<<std::endl;
    //std::cout<< std::setprecision(15)<<"pubtime:"<<pubtime<<std::endl;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped OdomBasefootprintTransMsg_;
    OdomBasefootprintTransMsg_.header.stamp = ros::Time::now();
    OdomBasefootprintTransMsg_.header.frame_id = "odom";
    OdomBasefootprintTransMsg_.child_frame_id = "base_foot";

    OdomBasefootprintTransMsg_.transform.translation.x = odom_akerman.pose.pose.position.x;
    OdomBasefootprintTransMsg_.transform.translation.y = odom_akerman.pose.pose.position.y;
    OdomBasefootprintTransMsg_.transform.translation.z = 0.0;
    OdomBasefootprintTransMsg_.transform.rotation = odom_akerman.pose.pose.orientation;
    br.sendTransform(OdomBasefootprintTransMsg_);


    output_count_ = output_count_ + 1;
}

int main(int argc, char **argv)
{
    g_filter_akerman.setModelParameter(AKERMANModelParameter(1,10)); //sigma2_beta, sigma2_a, l

    ros::init(argc, argv, "akermangpsvel");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    odom_akermanbs_pub = node.advertise<nav_msgs::Odometry>("/akermangpsvelodometry", 10000);//pub出估计的结果

    ros::Subscriber GPSPOS_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/fixtoodometry", 10000, gpsCallback);//10hz
    //ros::Subscriber GPSPOS_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/filteredodometry", 10000, gpsCallback);//10hz
    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 10, imuCallback);//200hz
    ros::Subscriber SWHANG_sub = node.subscribe("/vehicle/steering_report", 10000,swhangCallback);//轮速，转角 100hz
    ros::Subscriber WHEEL_sub = node.subscribe("/vehicle/wheel_speed_report", 10000,wheelCallback);//轮速，转角 100hz

    ros::spin();//循环
}