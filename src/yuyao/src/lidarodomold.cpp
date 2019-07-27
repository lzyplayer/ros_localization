/**
 * 使用自行车模型，适用于林肯车
 * 融合加速度计，陀螺仪　加速度的观测只是简单对应;没有方向盘转角、轮速信息
 * 激光里程计信息及Alexander算法处理延迟
 * 
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
// #include <yuyao/Zpre.h>
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
//#include "ekf_core/measurements/steeringwheel_measurement.hpp"
//#include "ekf_core/measurements/pitch_measurement.hpp"

#include "ekf_core/models/bicycle.hpp"

#include "ekf_core/extended_kalman_filter.hpp"

using namespace std;
using namespace chrono;//计时用的

PositionMeasurement g_slam_position_measurement;//在measurements里定义的类
HeadingMeasurement g_slam_heading_measurement;
AngularvelocityMeasurement g_imu_angularvelocity_measurement;
AccelerationMeasurement g_imu_acceleration_measurement;
VelocityMeasurement g_lidar_velocity_measurement;
AngularvelocityMeasurement g_lidar_angularvelocity_measurement;

bool g_slam_measurement_changed;
bool g_lidar_measurement_changed;

bool inite_g_slam_measurement = false;
bool inite_g_lidar_measurement = false;

ExtendedKalmanFilter<BICYCLEModel> g_filter_bicycle;

double rostimebegin;
double rostimeend;
double rostimepub;
double oldpub = 0;
double deltapub = 0;
double afterpub = 0;
double pubtime = 0;
double z_=0;

int output_count_ = 0;
int N = 0;
//传感器本身给定的误差水平,也许可以改成自适应
//slam信号弱情况下，要提高其不准确度，根据covar可以输出体现
double covar_slamyaw=0.04;//双天线朝向结果，会有NAN情况
double covar_imuacc=0.01;
double covar_imuang=0.01;
double covar_lidarang=0.04;
double covar_lidarvel=0.5;

struct Zpre
{
    double v, w;//获取时的预测
    double stamp;//时间，为了验证
};
queue<Zpre> M;

double L=15.5;//方向盘转角比车轮转角/////////////////////////////////////////////////////////////////////////////
double w_limit=0.05;

bool inite = false;
void update(double timestamp);
nav_msgs::Odometry odom;//封装
ros::Publisher rawodombicycle_pub ,odom_bicycle_pub;


decltype(g_filter_bicycle)::Model::FilterVector outputstate_bicycle_;//实时输出
decltype(g_filter_bicycle)::Model::FilterVector state_;//雷达第二帧状态
double delta_;
double t2_;
int N_1 = 0;
int N_2=1;
int seq_duan = 1990;
double timestamp_old;
Eigen::Matrix<double, 1, 1> measurement1_;
Eigen::Matrix<double, 1, 1> measurement2_;

boost::mutex inputmutex;/////////////////////////////////////////////////////////////////////////////////////////////////、、、、、、、、、、
Eigen::IOFormat eigen_csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, "," , "," , "" , "" , "", "");

void slamCallback(const nav_msgs::OdometryConstPtr& input)// POS VEL YAW
{
    {/////////////////////////////////////////////////////////////slamtest//////////////////////////////////////
        //slam良好情况下人为去掉slam信号输入
        /*
        {
            N = N+1;  
            if(N>240)
            {
                std::cout << "断掉slam，算法还准吗" << std::endl;
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


        //入地库之前断掉slam信号输入,序列号是“diku”bag的下地库部分
        {//慢速提供
            int seq=input->header.seq;
            if(N_1>10)
            {
                if(N_1%4 != 0)
                {
                    std::cout << "断掉GPS咯" << std::endl;
                    return;
                }
                N_1 += 1;
            }
            
        }
        
    
        // int seq=input->header.seq;
        // if(seq>STAMP)
        // {
        //     std::cout << "断slam" << std::endl;
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
        //     std::cout << "filteredyaw:" << yaw <<std::endl;
        //     return;
        // }
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
            //slam POS
        double a = input->pose.pose.position.x;
        if(!std::isnan(a))//判断是否是nan
        {
            inite_g_slam_measurement = true;//slam测量

            Eigen::Matrix<double, 2, 1> measurement;
            measurement << input->pose.pose.position.x - 313896.082284, input->pose.pose.position.y - 3791429.257284;
            Eigen::Matrix<double, 2, 2> covariance;
            covariance << input->pose.covariance[0] * input->pose.covariance[0], 0,
            0,input->pose.covariance[1] * input->pose.covariance[1];/////////////////////////////////////////////////含义？？？？
            g_slam_position_measurement.setMeasurement(measurement, covariance, timestamp);
            z_=input->pose.pose.position.z;

            g_slam_measurement_changed = true;
            //std::cout<<"slampos:"<< measurement <<std::endl;
        }
        

        
        //slam YAW 使用的是原始双天线朝向，不完全输入，存在NAN
        a=input->pose.pose.orientation.w;
        if(!std::isnan(a))
        {//将四元数转换为欧拉角

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
            covariance << covar_slamyaw;
            g_slam_heading_measurement.setMeasurement(measurement, covariance, timestamp);

            std::cout<<"slamyaw:"<< yaw <<std::endl;
        }    
    }
}


// ////****ADD****////
void lidarsigCallback(const nav_msgs::OdometryConstPtr& input)//记录状态信息
{
    t2_=input->header.stamp.toSec();//bag时间戳,存储
    cout << "signal" << endl;

    int total_length_bicycle = 0;//先统计这次更新用多少维
    total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AccelerationMeasurement>();
    total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
    if(g_slam_measurement_changed) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<PositionMeasurement>();
    if(g_lidar_measurement_changed) 
    {
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<VelocityMeasurement>();
        total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
    }
    bool f = true;
    g_filter_bicycle.beginAddMeasurement(t2_, total_length_bicycle, f);

    Zpre Zs;
    Zs.v = g_filter_bicycle.state_pre_(3);//prediction;
    Zs.w = g_filter_bicycle.state_pre_(4);
    Zs.stamp = t2_;//点对第一帧时间戳
    M.push(Zs); 
    cout<<"**"<< M.size() <<"**"<<endl;

}
//保证ｓｋ对齐机制，不要漏帧错帧　　附带当时的时间戳
void lidarCallbacknew(const nav_msgs::OdometryConstPtr& input)// 处理演化过程
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
            ** bicycle State Predition by f(State)　使用第二帧对应时刻的位姿和其他状态信息，预测到目前时刻
            \*--------------------------------------------------------------------*/
        if(!M.empty())
        {
            Zpre n;//用来记录读出来的东西； 
            n=M.front();
            double t1 = n.stamp;
            M.pop();
            Zpre n_new;
            n_new = M.front();
            double delta = n_new.stamp - t1;
            cout<<"*%%*"<< M.size() <<"**"<<endl;
            std::cout << std::setprecision(15) << "后一个:" << n_new.stamp << std::endl;
            std::cout << std::setprecision(15) << "delta:" << delta << std::endl;
            std::cout << std::setprecision(15) << "栈顶" << n.stamp <<std::endl;
            std::cout << std::setprecision(15) << "输入" << timestamp_old <<std::endl;
            while(n_new.stamp != timestamp_old)//可能出现sig在结果之后吗？序列顺序？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
            {
                n = M.front();
                t1 = n.stamp;
                M.pop();
                delta = M.front().stamp - t1;
                std::cout << std::setprecision(15) << "不对齐了ｎ（＊≧▽≦＊）ｎ" <<std::endl;
                std::cout << std::setprecision(15) << "栈顶" << n.stamp <<std::endl;
                std::cout << std::setprecision(15) << "输入" << timestamp_old <<std::endl;
            }
            double a = input->pose.pose.position.x;
            if(!std::isnan(a))//判断是否是nan
            {
                inite_g_lidar_measurement = true;//slam测量
                Eigen::Matrix<double, 1, 1> measurement;
                double X_ = input->pose.pose.position.x;
                double Y_ = input->pose.pose.position.y;
                double vv = (double)sqrt(X_*X_+Y_*Y_)/delta;
                measurement << vv - n.v;
                //std::cout << "LIDAR位置方差=" << measurement << std::endl;
                Eigen::Matrix<double, 1, 1> covariance;
                covariance << covar_lidarvel;
                g_lidar_velocity_measurement.setMeasurement(measurement, covariance, timestamp);
                measurement1_ = measurement;

                Eigen::Matrix<double, 1, 1> measurement1;
                measurement1 << yaw/delta - n.w;
                Eigen::Matrix<double, 1, 1> covariance1;
                covariance1 << covar_lidarang;
                g_lidar_angularvelocity_measurement.setMeasurement(measurement1, covariance1, timestamp);
                measurement2_ = measurement1;

                g_lidar_measurement_changed = true;
            }
        }
    }
}


void imuCallback(const sensor_msgs::ImuConstPtr& input)
{
    rostimebegin = ros::Time::now().toSec();///////////////////////////////////////////////////////////////////////////////////////////////时间对齐
    //std::cout<< std::setprecision(15)<<"rostimebegin:"<<rostimebegin<<std::endl;//precision控制精度
    auto start = system_clock::now();//计时
    //cout << "update" << endl;
    bool imuinite = true;
    double timestamp = input->header.stamp.toSec();
    //cout << std::setprecision(15) <<"timestamp" << timestamp << endl;

    {
        //boost::mutex::scoped_lock lock(inputmutex);
        if(!inite)//先获取ＧＰＳ信息
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

            Eigen::Matrix<double, 3, 3> W;
            W << 0.0329983, -0.999447, 0.00401395,
                -0.999455,  -0.0330029, -0.00110399,
                 0.00123585, -0.00397533, -0.999991;
            Eigen::Matrix<double, 3, 1> acc;
            acc << input->linear_acceleration.x, input->linear_acceleration.y, input->linear_acceleration.z;
            acc = W* acc;

            Eigen::Matrix<double, 1, 1> accelerationmeasurement;
            accelerationmeasurement << acc(0);
            Eigen::Matrix<double, 1, 1> accelerationcovariance;
            accelerationcovariance << covar_imuacc;
            g_imu_acceleration_measurement.setMeasurement(accelerationmeasurement, accelerationcovariance, timestamp);
        }
    }
    //std::cout<<"imuinite"<<imuinite<<inite_g_slam_position_measurement<<inite_g_slam_velocity_measurement<<inite_g_slam_heading_measurement<<std::endl;
    //if(imuinite&&inite_g_slam_position_measurement&&inite_g_slam_heading_measurement)
    if(imuinite&&inite_g_slam_measurement)
    //if(imuinite)//执行条件
    {
        odom.header.stamp=input->header.stamp;
        update(timestamp);

    }

    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    //cout<< std::setprecision(15) <<  "花费了spend" << double(duration.count()) * microseconds::period::num / microseconds::period::den << "ssss秒" << endl;

    rostimeend = ros::Time::now().toSec();
    //std::cout<< std::setprecision(15)<<"耗时:"<<rostimeend - rostimebegin<<std::endl;
}

void update(double timestamp)//来什么量测，用什么来更新
{

    //bool use_slam = g_slam_position_measurement_changed && g_slam_heading_measurement_changed;
    bool use_slam = g_slam_measurement_changed;
    bool use_lidar = g_lidar_measurement_changed;

#ifdef DEBUG
    double last_w = 0;
    std::cout << "USAGE: IMU | ";
    if(use_slampos) std::cout<< "slamPOS" << " | ";
    if(use_slamyaw) std::cout<< "slamYAW" << " | ";
    if(use_slamvel) std::cout<< "slamVEL" << " | ";
    if(use_vehvel) std::cout<< "VEHVEL" << " | ";
    if(use_swangle) std::cout<< "SWANGLE" << " | ";
    std::cout << std::endl;
#endif
    {
        boost::mutex::scoped_lock lock(inputmutex);////////////////////////////////////////////////////////////////////////////////////////////////////
        {
            int total_length_bicycle = 0;//先统计这次更新用多少维
            total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AccelerationMeasurement>();
            total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
            if(use_slam) 
            {
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<PositionMeasurement>();
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<HeadingMeasurement>();
            }
            if(use_lidar) 
            {
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<VelocityMeasurement>();
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
            }
        //std::cout<<"totallength:" << total_length;
            bool f = false;
            g_filter_bicycle.beginAddMeasurement(timestamp, total_length_bicycle, f);
            g_filter_bicycle.AddMeasurement(g_imu_angularvelocity_measurement);
            g_filter_bicycle.AddMeasurement(g_imu_acceleration_measurement);
            if(use_slam) 
            {
                g_filter_bicycle.AddMeasurement(g_slam_position_measurement);
                g_filter_bicycle.AddMeasurement(g_slam_heading_measurement);
            }
            //处理
            if(use_lidar)//对量测进行二次推演，改变
            {
                Eigen::Matrix<double, 1, 1> measurement1;
                Eigen::Matrix<double, 1, 1> measurement2;
                
                measurement1 << g_filter_bicycle.state_pre_(3);
                g_lidar_velocity_measurement.changeMeasurement(measurement1);
                g_filter_bicycle.AddMeasurement(g_lidar_velocity_measurement);

                measurement2 << g_filter_bicycle.state_pre_(4);
                g_lidar_angularvelocity_measurement.changeMeasurement(measurement2);
                g_filter_bicycle.AddMeasurement(g_lidar_angularvelocity_measurement);
            }
        }
        if(use_slam) g_slam_measurement_changed = false;
        if(use_lidar) g_lidar_measurement_changed = false;
    }
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////covarience????????????????

    decltype(g_filter_bicycle)::Model::FilterVector outputstate_bicycle;
    decltype(g_filter_bicycle)::Model::FilterMatrix outputcovariance_bicycle;
    double outputtime_bicycle;

    g_filter_bicycle.endAddMeasurement(outputstate_bicycle, outputcovariance_bicycle, outputtime_bicycle);

    std::cout << "P = "<< outputcovariance_bicycle <<std::endl;
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
    odom_bicycle.pose.pose.position.y = position_bicycle(1)+3791429.257284 + 1.02* sin(outputstate_bicycle(2));
    odom_bicycle.pose.pose.position.z = z_;
    Eigen::Matrix<double, 3, 1> t_bicycle_;
    t_bicycle_ << position_bicycle(0) + 1.02* cos(outputstate_bicycle(2)),
                  position_bicycle(1) + 1.02* sin(outputstate_bicycle(2)),
                  z_;
    
    tf2::Quaternion quat_bicycle_;
    quat_bicycle_.setRPY(0, 0, heading_bicycle(0));
    cout << "朝向:" << heading_bicycle(0) << endl;
    odom_bicycle.pose.pose.orientation.x = quat_bicycle_.x();
    odom_bicycle.pose.pose.orientation.y = quat_bicycle_.y();
    odom_bicycle.pose.pose.orientation.z = quat_bicycle_.z();
    odom_bicycle.pose.pose.orientation.w = quat_bicycle_.w();

    outputstate_bicycle_=outputstate_bicycle;//再倒到雷达数据来的那个时候

    //velocity_bicycle(0)+=0.1;
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
    if(use_slampos) cout << "REF: " << g_slam_position_measurement.measurement_(0) << ", \t" << g_slam_position_measurement.measurement_(1);
    cout << endl;

    cout << "  TW : " << outputstate(2) << ", \t" << outputstate(4) << ", \t";
    cout << "REF: ";
    if(use_slamyaw) cout << g_slam_heading_measurement.measurement_(0) <<  ", \t";
    else cout << "---, \t";
    cout << g_imu_angularvelocity_measurement.measurement_(0) << endl;

    cout << "  VA : " << outputstate(3) << ", \t" << outputstate(5) << ", \t";
    cout << "REF: ";
    if(use_slamvel) cout << g_slam_velocity_measurement.measurement_(0) <<  ", \t";
    else cout << "---, \t";
    cout << g_imu_acceleration_measurement.measurement_(0) << endl;

    //if(abs(outputstate(4)) > 1) throw;
    if(abs(outputstate(4) - last_w) > 1) throw;
    //if(abs(outputstate(5)) > 0.5) throw;
    if(use_slampos && abs(outputstate(0) - g_slam_position_measurement.measurement_(0)) > 3) throw;
    if(use_slampos && abs(outputstate(1) - g_slam_position_measurement.measurement_(1)) > 3) throw;
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
    g_filter_bicycle.setModelParameter(BICYCLEModelParameter(10,10,1.25,1.6)); //sigma2_beta, sigma2_a, l

    ros::init(argc, argv, "localizationbicycle");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    odom_bicycle_pub = node.advertise<nav_msgs::Odometry>("/yuyaolidarodometry", 10000);//pub出估计的结果

    // ros::Subscriber SLAMPOS_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/filteredodometry", 10000, slamCallback);//10hz
    ros::Subscriber SLAMPOS_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/fixtoodometry", 10000, slamCallback);//10hz
    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 10, imuCallback);//200hz
    ros::Subscriber LIDAR_sub = node.subscribe("/lidar_odom", 10000, lidarCallbacknew);//10hz
    ros::Subscriber LIDARSIG_sub = node.subscribe("/lidar_stamp", 10000, lidarsigCallback);//10hz

    ros::spin();//循环
}//看闭合结果