//算法说明
//BICYCLE模型，
//量测使用SLAM位置、SLAM朝向、IMU、方向盘转角、轮速计、激光里程计
//对齐方式是按IMU，但没有零阶保持

#include <string>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iterator>
#include <chrono>
#include <queue>
#include <ros/ros.h>
//msgs
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <dbw_mkz_msgs/SteeringReport.h>
//#include <dbw_mkz_msgs/WheelSpeedReport.h>
#include <message_filters/subscriber.h>
//tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>/////////////////////tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Transform.h>
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
//core
#include "ekf_core/measurements/position_measurement.hpp"
#include "ekf_core/measurements/velocity_measurement.hpp"
#include "ekf_core/measurements/heading_measurement.hpp"
#include "ekf_core/measurements/angularvelocity_measurement.hpp"
#include "ekf_core/measurements/acceleration_measurement.hpp"
#include "ekf_core/measurements/steeringwheel_measurement.hpp"
//#include "ekf_core/measurements/wheel_measurement.hpp"
#include "ekf_core/models/bicycle.hpp"
#include "ekf_core/extended_kalman_filter.hpp"

using namespace std;
using namespace chrono;//计时用的
using Eigen::MatrixXd;

PositionMeasurement g_slam_position_measurement;//在measurements里定义的类
HeadingMeasurement g_slam_heading_measurement;
AngularvelocityMeasurement g_imu_angularvelocity_measurement;
AccelerationMeasurement g_imu_acceleration_measurement;
SteeringwheelangleMeasurement g_sw_angle_measurement;//方向盘转角
VelocityMeasurement g_lidar_velocity_measurement;
AngularvelocityMeasurement g_lidar_angularvelocity_measurement;

bool g_slam_measurement_changed;
bool g_sw_angle_measurement_changed;
bool g_lidar_measurement_changed;;
//bool use_imupitch;

bool inite_g_slam_measurement = false;
bool inite_g_sw_angle_measurement = false;
bool inite_g_lidar_measurement = false;

ExtendedKalmanFilter<BICYCLEModel> g_filter_bicycle;

double rostimebegin;
double rostimeend;
// double rostimepub;
// double deltapub = 0;
// double pubtime = 0;
double z_=0;

int output_count_ = 0;
int N = 0;
double v_=0.02;
double w_=0.01;

//////            R          //////
//传感器本身给定的误差水平,也许可以改成自适应
//SLAM信号弱情况下，要提高其不准确度，根据covar可以输出体现
MatrixXd covar_slampos = MatrixXd::Identity(2, 2) * 0.1;
double covar_slamyaw = 0.04;
double covar_swang=0.002;//不太准确
double covar_imuacc=0.04;
double covar_imuang=0.04;
double covar_lidarang=0.09;
double covar_lidarvel=0.5;
////        parameter           //////
int seq_duan = 2100;//调试，模拟GPS失锁
double K=15.5;//方向盘转角比车轮转角
double per = 0.96;//测量轮角速度到轮速的误差比例，半径误差
//轮速信息使用条件

struct Zpre_lidar
{
    double v, w;//获取时的预测,速度，角速度，位置，朝向
    double stamp;//时间，为了验证
};
struct Zpre_slam
{
    double x, y, t;//获取时的预测,速度，角速度，位置，朝向
    double stamp;//时间，为了验证
};
queue<Zpre_slam> M_slam;
queue<Zpre_lidar> M_lidar;
Zpre_slam n_slam;
Zpre_lidar n_lidar;
decltype(g_filter_bicycle)::Model::FilterVector outputstate_bicycle_;//实时输出
decltype(g_filter_bicycle)::Model::FilterVector state_;//雷达第二帧状态
double delta_;
double t2_;
double timestamp_old;
Eigen::Matrix<double, 2, 1> measurement1_slam;//pos
Eigen::Matrix<double, 1, 1> measurement1_lidar;//yaw
Eigen::Matrix<double, 1, 1> measurement2_slam;//v
Eigen::Matrix<double, 1, 1> measurement2_lidar;//w

bool inite = false;
void update(double timestamp, ros::Time stamp);
ros::Publisher rawodombicycle_pub ,odom_bicyclebs_pub;
string map_tf;
string base_lidar_tf;

boost::mutex inputmutex;/////////////////////////////////////////////////////////////////////////////////////////////////、、、、、、、、、、

Eigen::IOFormat eigen_csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, "," , "," , "" , "" , "", "");

void SlamsigCallback(const nav_msgs::OdometryConstPtr& input)//记录状态信息
{
    t2_=input->header.stamp.toSec();//bag时间戳,存储
    cout << "slam_signal" << endl;

    int total_length_bicycle = 0;//先统计这次更新用多少维
    total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AccelerationMeasurement>();
    total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
    if(g_slam_measurement_changed)
    {
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<PositionMeasurement>();
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<HeadingMeasurement>();
    } 
    if(g_sw_angle_measurement_changed) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<SteeringwheelangleMeasurement>();
    if(g_lidar_measurement_changed) 
    {
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<VelocityMeasurement>();
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
    }
    bool f = true;
    g_filter_bicycle.beginAddMeasurement(t2_, total_length_bicycle, f);

    Zpre_slam Zs;
    Zs.x = g_filter_bicycle.state_pre_(0);//prediction;
    Zs.y = g_filter_bicycle.state_pre_(1);
    Zs.t = g_filter_bicycle.state_pre_(2);
    Zs.stamp = t2_;//获取时间

    M_slam.push(Zs); 
    cout<<"**M_slam"<< M_slam.size() <<"**"<<endl;

}

//保证ｓｋ对齐机制，不要漏帧错帧　　附带当时的时间戳
void SlamCallback(const nav_msgs::OdometryConstPtr& input)//两个时间戳，bag原始的时间戳以及计算时间长度，得到解算完的对应bag的时间
{
    double timestamp = input->header.stamp.toSec() + input->twist.twist.linear.x;//处理后的时间
    timestamp_old = input->header.stamp.toSec();//点云获取时间
    if(!inite)//模型初始化
    {
        g_filter_bicycle.init(timestamp);
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
        if(!M_slam.empty())
        {
            Zpre_slam n;//用来记录读出来的东西； 
            n = M_slam.front();
            M_slam.pop();
            Zpre_slam n_new;
            n_new = M_slam.front();
            cout<<"*%%M_slam*"<< M_slam.size() <<"**"<<endl;
            std::cout << std::setprecision(15) << "SLAM后一个:" << n_new.stamp << std::endl;
            std::cout << std::setprecision(15) << "SLAM栈顶" << n.stamp <<std::endl;
            std::cout << std::setprecision(15) << "SLAM输入" << timestamp_old <<std::endl;
            while(n.stamp != timestamp_old)//可能出现sig在结果之后吗？序列顺序？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
            {
                std::cout << std::setprecision(15) << "SLAM不对齐了ｎ（＊≧▽≦＊）ｎ" <<std::endl;
                std::cout << std::setprecision(15) << "SLAM栈顶" << n.stamp <<std::endl;
                std::cout << std::setprecision(15) << "SLAM输入" << timestamp_old <<std::endl;
                if(n.stamp > timestamp_old)
                {
                    return;
                }
                else
                {
                    n = M_slam.front();
                    M_slam.pop();
                }
            }

            double a = input->pose.pose.position.x;
            if(!std::isnan(a))//判断是否是nan
            {
                inite_g_slam_measurement = true;//slam测量

                Eigen::Matrix<double, 2, 1> measurement;
                double X_ = input->pose.pose.position.x;
                double Y_ = input->pose.pose.position.y;
                z_ = input->pose.pose.position.z;

                measurement << X_ - n.x, Y_ - n.y;
                Eigen::Matrix<double, 2, 2> covariance;
                covariance = covar_slampos;
            //     covariance << input->pose.covariance[0] * input->pose.covariance[0], 0,
            // 0,input->pose.covariance[1] * input->pose.covariance[1];/////////////////////////////////////////////////含义？？？？
                g_slam_position_measurement.setMeasurement(measurement, covariance, timestamp);
                measurement1_slam = measurement;

                Eigen::Matrix<double, 1, 1> measurement1;
                measurement1 << yaw - n.t;
                Eigen::Matrix<double, 1, 1> covariance1;
                covariance1 << covar_slamyaw;
                g_slam_heading_measurement.setMeasurement(measurement1, covariance1, timestamp);
                measurement2_slam = measurement1;

                g_slam_measurement_changed = true;
            }
        }
    }
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
            measurement << input->steering_wheel_angle/K;//K是转角比例
            Eigen::Matrix<double, 1, 1> swanglecovariance;
            swanglecovariance << covar_swang;
            g_sw_angle_measurement.setMeasurement(measurement, swanglecovariance, timestamp);//写入成员变量

            g_sw_angle_measurement_changed = true;
            //std::cout<<"VEHangle:"<< measurement <<std::endl;
        }
    }
}


void LidarsigCallback(const nav_msgs::OdometryConstPtr& input)//记录状态信息
{
    t2_=input->header.stamp.toSec();//bag时间戳,存储
    cout << "lidar_signal" << endl;

    int total_length_bicycle = 0;//先统计这次更新用多少维
    total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AccelerationMeasurement>();
    total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
    if(g_slam_measurement_changed)
    {
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<PositionMeasurement>();
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<HeadingMeasurement>();
    } 
    if(g_sw_angle_measurement_changed) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<SteeringwheelangleMeasurement>();
    if(g_lidar_measurement_changed) 
    {
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<VelocityMeasurement>();
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
    }
    bool f = true;
    g_filter_bicycle.beginAddMeasurement(t2_, total_length_bicycle, f);

    Zpre_lidar Zs;
    Zs.v = g_filter_bicycle.state_pre_(3);//prediction;
    Zs.w = g_filter_bicycle.state_pre_(4);
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
        g_filter_bicycle.init(timestamp);
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

                Eigen::Matrix<double, 1, 1> measurement;
                double X_ = input->pose.pose.position.x;
                double Y_ = input->pose.pose.position.y;
                double vv = (double)sqrt(X_*X_+Y_*Y_)/delta;
                measurement << vv - n.v;
                Eigen::Matrix<double, 1, 1> covariance;
                covariance << covar_lidarvel;
                g_lidar_velocity_measurement.setMeasurement(measurement, covariance, timestamp);
                measurement1_lidar = measurement;

                Eigen::Matrix<double, 1, 1> measurement1;
                measurement1 << yaw/delta - n.w;
                Eigen::Matrix<double, 1, 1> covariance1;
                covariance1 << covar_lidarang;
                g_lidar_angularvelocity_measurement.setMeasurement(measurement1, covariance1, timestamp);
                measurement2_lidar = measurement1;

                g_lidar_measurement_changed = true;
            }
        }
    }
}

void imuCallback(const sensor_msgs::ImuConstPtr& input)
{
    rostimebegin = ros::Time::now().toSec();///////////////////////////////////////////////////////////////////////////////////////////////时间对齐
    //std::cout<< std::setprecision(15)<<"rostimebegin:"<<rostimebegin<<std::endl;

    bool imuinite = true;
    double timestamp = input->header.stamp.toSec();
    ros::Time stamp=input->header.stamp;
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
    if(imuinite&&inite_g_slam_measurement)
    //if(imuinite)//执行条件
    {
        update(timestamp,stamp);
    }
    rostimeend = ros::Time::now().toSec();
    std::cout<< std::setprecision(15)<<"imu update 花费:"<<rostimeend - rostimebegin<<std::endl;
}

void update(double timestamp, ros::Time stamp)//来什么量测，用什么来更新
{
    //std::cout<<"更新开始" <<std::endl;
    bool use_slam = g_slam_measurement_changed;
    bool use_swangle = g_sw_angle_measurement_changed;
    bool use_lidar = g_lidar_measurement_changed;

    {
        boost::mutex::scoped_lock lock(inputmutex);////////////////////////////////////////////////////////////////////////////////////////////////////
       //bicycle
        {
            int total_length_bicycle = 0;//先统计这次更新用多少维
            total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AccelerationMeasurement>();
            total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
            if(use_slam) 
            {
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<PositionMeasurement>();
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<HeadingMeasurement>();
            }
            if(use_swangle) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<SteeringwheelangleMeasurement>();
            if(use_lidar) 
            {
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<VelocityMeasurement>();
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
            }

            bool f = false;
            g_filter_bicycle.beginAddMeasurement(timestamp, total_length_bicycle, f);
            g_filter_bicycle.AddMeasurement(g_imu_angularvelocity_measurement);
            g_filter_bicycle.AddMeasurement(g_imu_acceleration_measurement);
            if(use_slam) 
            {
                Eigen::Matrix<double, 2, 1> measurement1;
                Eigen::Matrix<double, 1, 1> measurement2;

                measurement1 << g_filter_bicycle.state_pre_(0), g_filter_bicycle.state_pre_(1);
                g_slam_position_measurement.changeMeasurement(measurement1);
                g_filter_bicycle.AddMeasurement(g_slam_position_measurement);
                measurement1_slam += measurement1;
                std::cout<<"SLAMpos:"<< measurement1_slam <<std::endl;

                measurement2 << g_filter_bicycle.state_pre_(2);
                g_slam_heading_measurement.changeMeasurement(measurement2);
                g_filter_bicycle.AddMeasurement(g_slam_heading_measurement);
                measurement2_slam += measurement2;
                std::cout<<"SLAMheading:"<< measurement2_slam <<std::endl;
            }
            if(use_swangle) g_filter_bicycle.AddMeasurement(g_sw_angle_measurement);
            if(use_lidar)//对量测进行二次推演，改变
            {            
                Eigen::Matrix<double, 1, 1> measurement1;
                Eigen::Matrix<double, 1, 1> measurement2;
                
                measurement1 << g_filter_bicycle.state_pre_(3);
                g_lidar_velocity_measurement.changeMeasurement(measurement1);
                g_filter_bicycle.AddMeasurement(g_lidar_velocity_measurement);
                measurement1_lidar += measurement1;
                std::cout<<"LIDARvel:"<< measurement1_lidar <<std::endl;

                measurement2 << g_filter_bicycle.state_pre_(4);
                g_lidar_angularvelocity_measurement.changeMeasurement(measurement2);
                g_filter_bicycle.AddMeasurement(g_lidar_angularvelocity_measurement);
                measurement2_lidar += measurement2;
                std::cout<<"LIDARangular:"<< measurement2_lidar <<std::endl;
            }
        }

        if(use_slam) g_slam_measurement_changed = false;
        if(use_swangle) g_sw_angle_measurement_changed = false;
        if(use_lidar) g_lidar_measurement_changed = false;
    }
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////covarience????????????????

    decltype(g_filter_bicycle)::Model::FilterVector outputstate_bicycle;
    decltype(g_filter_bicycle)::Model::FilterMatrix outputcovariance_bicycle;
    double outputtime_bicycle;

    g_filter_bicycle.endAddMeasurement(outputstate_bicycle, outputcovariance_bicycle, outputtime_bicycle);

    std::cout << "P = "<< outputcovariance_bicycle <<std::endl;
    std::cout << "状态 = "<< outputstate_bicycle <<std::endl;

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

    odom_bicycle.header.stamp = stamp;
    // odom_bicycle.header.frame_id = "odom";
    // odom_bicycle.child_frame_id = "base_foot";
    odom_bicycle.header.frame_id = "map_tf";
    odom_bicycle.child_frame_id = "base_lidar_tf";

    odom_bicycle.pose.pose.position.x = position_bicycle(0) + 313896.082284;//激光位置
    odom_bicycle.pose.pose.position.y = position_bicycle(1) + 3791429.257284;
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

    v_=outputstate_bicycle(3);
    w_=outputstate_bicycle(4);
    //std::cout<<"速度"<< v_ <<std::endl;
    //std::cout<<"角速度"<< w_ <<std::endl;
    odom_bicyclebs_pub.publish(odom_bicycle);
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


    output_count_ = output_count_ + 1;
}

int main(int argc, char **argv)
{
    g_filter_bicycle.setModelParameter(BICYCLEModelParameter(0.1, 0.5, 1.25, 1.6)); //sigma2_beta, sigma2_a, l

    ros::init(argc, argv, "bicycleslamlidar");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    map_tf = priv_node.param<std::string>("map_tf", "map");
    base_lidar_tf = priv_node.param<std::string>("base_lidar_tf", "velodyne");

    odom_bicyclebs_pub = node.advertise<nav_msgs::Odometry>("/bicycleslamlidarodometry", 10000);//pub出估计的结果

    ros::Subscriber SLAMPOS_sub = node.subscribe("/odom", 10000, SlamCallback);
    ros::Subscriber SLAMSIG_sub = node.subscribe("/stamp", 10000, SlamsigCallback);
    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 10, imuCallback);
    ros::Subscriber SWHANG_sub = node.subscribe("/vehicle/steering_report", 10000,swhangCallback);//转角
    ros::Subscriber LIDAR_sub = node.subscribe("/lidar_odom", 10000, LidarCallback);
    ros::Subscriber LIDARSIG_sub = node.subscribe("/lidar_stamp", 10000, LidarsigCallback);

    ros::spin();//循环
}//看闭合结果
