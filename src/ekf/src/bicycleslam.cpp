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
#include <memory>
#include <ros/ros.h>
//msgs
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/WheelSpeedReport.h>
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
#include <tf/transform_datatypes.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

//core
#include "ekf_core/measurements/position_measurement.hpp"
#include "ekf_core/measurements/velocity_measurement.hpp"
#include "ekf_core/measurements/heading_measurement.hpp"
#include "ekf_core/measurements/angularvelocity_measurement.hpp"
#include "ekf_core/measurements/acceleration_measurement.hpp"
#include "ekf_core/measurements/steeringwheel_measurement.hpp"
#include "ekf_core/measurements/wheel_measurement.hpp"
#include "ekf_core/models/bicycle.hpp"
#include "ekf_core/extended_kalman_filter.hpp"
#include <transform_utility.hpp>


using namespace std;
using namespace chrono;//计时用的
using Eigen::MatrixXd;

PositionMeasurement g_slam_position_measurement;//在measurements里定义的类
VelocityMeasurement g_slam_velocity_measurement;
HeadingMeasurement g_slam_heading_measurement;
AngularvelocityMeasurement g_imu_angularvelocity_measurement;
AccelerationMeasurement g_imu_acceleration_measurement;
SteeringwheelangleMeasurement g_sw_angle_measurement;//方向盘转角
VelocityMeasurement g_wheel_velocity_measurement;
AngularvelocityMeasurement g_wheel_angularvelocity_measurement;
VelocityMeasurement g_lidar_velocity_measurement;
AngularvelocityMeasurement g_lidar_angularvelocity_measurement;

bool g_slam_measurement_changed;
bool g_sw_angle_measurement_changed;
bool g_lidar_measurement_changed;
bool g_wheel_measurement_changed;
//bool use_imupitch;

bool inite_g_slam_measurement = false;
bool inite_g_sw_angle_measurement = false;
bool inite_g_lidar_measurement = false;
bool inite_g_wheel_measurement = false;

ExtendedKalmanFilter<BICYCLEModel> g_filter_bicycle;

double rostimebegin;
double rostimeend;
double z_=0;

int output_count_ = 0;
int N_ = -1;
double x_;
double y_;
double v_=0.01;
double w_=0.001;

//////            R          //////
//传感器本身给定的误差水平,也许可以改成自适应
//SLAM信号弱情况下，要提高其不准确度，根据covar可以输出体现
double cov_slampos, cov_imuacc, cov_imuang, cov_slamyaw, cov_slamvel, cov_wheel;
string frame_id, child_frame_id;
double pbeta, pa;
bool flag_slamv, flag_linearacc, flag_cali_imu, flag_slam_insout;
double cov_swang;//不太准确
double cov_lidarang=0.04;
double cov_lidarvel=0.5;
////        parameter           //////
double lr, lf;
int seq_duan = 2100;//调试，模拟GPS失锁
double K;//方向盘转角比车轮转角
double per = 0.96;//测量轮角速度到轮速的误差比例，半径误差
//轮速信息使用条件

struct Zpre_lidar
{
    double v, w;//获取时的预测,速度，角速度，位置，朝向
    double stamp;//时间，为了验证
};
queue<Zpre_lidar> M_lidar;
Zpre_lidar n_lidar;

decltype(g_filter_bicycle)::Model::FilterVector outputstate_bicycle_;//实时输出
decltype(g_filter_bicycle)::Model::FilterVector state_;//雷达第二帧状态
double delta_;
double t2_;
double timestamp_old;
bool flag_zero = false;
Eigen::Matrix<double, 1, 1> measurement1_lidar;//yaw
Eigen::Matrix<double, 1, 1> measurement2_lidar;//w

bool inite = false;
void update(double timestamp, ros::Time stamp);
ros::Publisher odom_bicyclebs_pub;

boost::mutex inputmutex;/////////////////////////////////////////////////////////////////////////////////////////////////、、、、、、、、、、

Eigen::IOFormat eigen_csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, "," , "," , "" , "" , "", "");
//tf pu
std::unique_ptr<tf2_ros::TransformBroadcaster> br;
// std::unique_ptr<tf2_ros::TransformListener> tr;

//保证ｓｋ对齐机制，不要漏帧错帧　　附带当时的时间戳
void SlamCallback(const nav_msgs::OdometryConstPtr& input)//两个时间戳，bag原始的时间戳以及计算时间长度，得到解算完的对应bag的时间
{
    double timestamp = input->header.stamp.toSec();
    
    inite = true;
    //慢速提供
    N_ += 1;
    // if(N_%10 != 0)
    // {
    //     std::cout << "断掉GPS咯" << std::endl;
    //     return;
    // }
    // if(N_>3000) return;
    
    {
        double a = input->pose.pose.position.x;
        if(!std::isnan(a))//判断是否是nan
        {
            Eigen::Matrix<double, 2, 1> measurement;
            // measurement << input->pose.pose.position.x - 313896.082284, input->pose.pose.position.y - 3791429.257284;
            measurement << input->pose.pose.position.x, input->pose.pose.position.y;
            z_ = input->pose.pose.position.z;
            Eigen::Matrix<double, 2, 2> covariance;
            covariance = Eigen::MatrixXd::Identity(2,2) * cov_slampos;
            // double cov = input->pose.covariance[0]/input->pose.covariance[1] * 0.1;//比例设定，适配于整个系统
            // covariance << cov, 0,
            // 0,cov;

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

            g_slam_position_measurement.setMeasurement(measurement, covariance, timestamp);
            
            if(N_ == 0)
            {
                g_filter_bicycle.initnew(timestamp, input->pose.pose.position.x, input->pose.pose.position.y, yaw);
                inite_g_slam_measurement = true;//GPS测量
                N_ += 1;
            }

            Eigen::Matrix<double, 1, 1> measurement1;
            measurement1 << yaw;
            Eigen::Matrix<double, 1, 1> covariance1;
            covariance1 << cov_slamyaw;
            g_slam_heading_measurement.setMeasurement(measurement1, covariance1, timestamp);
            
            if(flag_slamv)
            {
                Eigen::Matrix<double, 1, 1> measurement2;
                measurement2 << sqrt(input->twist.twist.linear.x * input->twist.twist.linear.x + input->twist.twist.linear.y * input->twist.twist.linear.y);
                Eigen::Matrix<double, 1, 1> covariance2;
                covariance2 << cov_slamvel;
                g_slam_heading_measurement.setMeasurement(measurement2, covariance2, timestamp);
            }
            g_slam_measurement_changed = true;
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
        return;
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
            swanglecovariance << cov_swang;
            g_sw_angle_measurement.setMeasurement(measurement, swanglecovariance, timestamp);//写入成员变量

            g_sw_angle_measurement_changed = true;
            std::cout<<"VEHangle:"<< measurement <<std::endl;
        }
    }
}


void LidarsigCallback(const nav_msgs::OdometryConstPtr& input)//记录状态信息
{
    if(!inite)//模型初始化
    {
        return;
    }
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
        return;
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
            n = M_lidar.front();
            double t1 = n.stamp;
            M_lidar.pop();
            double delta = timestamp - t1;
            cout<<"*%%M_lidar*"<< M_lidar.size() <<"**"<<endl;
            std::cout << std::setprecision(15) << "LIDAR delta:" << delta << std::endl;
            std::cout << std::setprecision(15) << "LIDAR栈顶" << n.stamp <<std::endl;
            std::cout << std::setprecision(15) << "LIDAR输入" << timestamp_old <<std::endl;
            while(n.stamp != timestamp_old)//因为第一次数据到，里程计是不输出的/////////////////////////////////////////////////////////////////
            {
                std::cout << std::setprecision(15) << "LIDAR不对齐了ｎ（＊≧▽≦＊）ｎ" <<std::endl;
                std::cout << std::setprecision(15) << "LIDAR栈顶" << n.stamp <<std::endl;
                std::cout << std::setprecision(15) << "LIDAR输入" << timestamp_old <<std::endl;
                if(n.stamp > timestamp_old)
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
                covariance << cov_lidarvel;
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
void wheelCallback(const dbw_mkz_msgs::WheelSpeedReportConstPtr& input)
{
        //boost::mutex::scoped_lock lock(inputmutex);
    double timestamp = input->header.stamp.toSec();

    if(!inite)
    {
        return;
    }
    else
    {
        double a = input->front_left;
        if(!std::isnan(a))
        {
            if(input->front_left == 0 && input->front_right == 0 && input->rear_left == 0 && input->rear_right == 0)
            {
                flag_zero = true;
            }
            if(v_==0 || w_==0)
            {
                return;
            }
            inite_g_wheel_measurement=true;
            Eigen::Matrix<double, 1, 1> wheelcovariance;
            wheelcovariance << Eigen::MatrixXd::Identity(1,1) * cov_wheel;

            Eigen::Matrix<double, 1, 1> measurement1;
            Eigen::Matrix<double, 1, 1> measurement2;
            measurement1 << 0.5* (input->rear_left + input->rear_right) * g_filter_bicycle.model_parameter_.R_w;
            measurement2 << (-input->rear_left + input->rear_right) * g_filter_bicycle.model_parameter_.R_w / g_filter_bicycle.model_parameter_.W;
            measurement1 *= per;
            measurement2 *= per;
            g_wheel_velocity_measurement.setMeasurement(measurement1, wheelcovariance, timestamp);//写入成员变量
            g_wheel_angularvelocity_measurement.setMeasurement(measurement2, wheelcovariance/10, timestamp);//写入成员变量
            g_wheel_measurement_changed = true;
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
            imuinite = false;
            return;
        }
        else
        {
            Eigen::Matrix<double, 1, 1> angularvelocitymeasurement;
            angularvelocitymeasurement << input->angular_velocity.z;
            Eigen::Matrix<double, 1, 1> angularvelocitycovariance;
            angularvelocitycovariance << cov_imuang;
            g_imu_angularvelocity_measurement.setMeasurement(angularvelocitymeasurement, angularvelocitycovariance, timestamp);
            
            Eigen::Matrix<double, 1, 1> accelerationmeasurement;
            accelerationmeasurement << input->linear_acceleration.x;
            // if(flag_linearacc)
            // {
            //     Eigen::Matrix<double, 1, 1> V;
            //     V << outputstate_bicycle_(4);
            //     accelerationmeasurement += angularvelocitymeasurement(0) * V;
            // }
            // if (flag_cali_imu)  
            // {
            //     Eigen::Matrix<double, 2, 2> C2;
            //     C2 << 0, -1,
            //           -1, 0;
            //     accelerationmeasurement = C2 * accelerationmeasurement;
            // }

            Eigen::Matrix<double, 1, 1> accelerationcovariance;
            accelerationcovariance << cov_imuacc;
            g_imu_acceleration_measurement.setMeasurement(accelerationmeasurement, accelerationcovariance, timestamp);
            // std::cout << "变换后x向加速度：" << accelerationmeasurement << std::endl;
            // std::cout << "另一种x向加速度：" << acc(0) << std::endl;
        }
    }
    if(imuinite&&inite_g_slam_measurement)
    {
        update(timestamp,stamp);
    }
    rostimeend = ros::Time::now().toSec();
    // std::cout<< std::setprecision(15)<<"imu update 花费:"<<rostimeend - rostimebegin<<std::endl;
}

void update(double timestamp, ros::Time stamp)//来什么量测，用什么来更新
{
    //std::cout<<"更新开始" <<std::endl;
    bool use_slam = g_slam_measurement_changed;
    bool use_swangle = g_sw_angle_measurement_changed;
    bool use_lidar = g_lidar_measurement_changed;
    bool use_wheel = g_wheel_measurement_changed;

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
                if(flag_slamv) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<VelocityMeasurement>();
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<HeadingMeasurement>();
            }
            if(use_swangle) total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<SteeringwheelangleMeasurement>();
            if(use_wheel) 
            {
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<VelocityMeasurement>();
                total_length_bicycle += decltype(g_filter_bicycle)::getLengthForMeasurement<AngularvelocityMeasurement>();
            }
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
                g_filter_bicycle.AddMeasurement(g_slam_position_measurement);
                g_filter_bicycle.AddMeasurement(g_slam_heading_measurement);
                if(flag_slamv) g_filter_bicycle.AddMeasurement(g_slam_velocity_measurement);
                // cout << "测量" << g_filter_bicycle.measurement_stack_ << endl << endl;
    
            }
            if(use_swangle) g_filter_bicycle.AddMeasurement(g_sw_angle_measurement);
            if(use_wheel) 
            {
                g_filter_bicycle.AddMeasurement(g_wheel_velocity_measurement);
                g_filter_bicycle.AddMeasurement(g_wheel_angularvelocity_measurement);
            }
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
        if(use_wheel) g_wheel_measurement_changed = false;
        // cout << "估计测量" << g_filter_bicycle.estimated_measurement_stack_ << endl << endl;

    }
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////covarience????????????????

    decltype(g_filter_bicycle)::Model::FilterVector outputstate_bicycle;
    decltype(g_filter_bicycle)::Model::FilterMatrix outputcovariance_bicycle;
    double outputtime_bicycle;

    g_filter_bicycle.endAddMeasurement(outputstate_bicycle, outputcovariance_bicycle, outputtime_bicycle, flag_zero);

    cout << "状态 = "<< outputstate_bicycle <<endl << endl;

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
    odom_bicycle.header.frame_id = frame_id;
    odom_bicycle.child_frame_id = child_frame_id;

    odom_bicycle.pose.pose.position.x = position_bicycle(0);//激光位置
    odom_bicycle.pose.pose.position.y = position_bicycle(1);
    // odom_bicycle.pose.pose.position.x = position_bicycle(0) + 313896.082284;//激光位置
    // odom_bicycle.pose.pose.position.y = position_bicycle(1) + 3791429.257284;
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
    if(!flag_slam_insout)
    {
        Eigen::Matrix4f transformationMatrix;
        // tf::StampedTransform ins2Velo;
        // try {

        //     tr->lookupTransform("velo_middle","ins_center",stamp,ins2Velo);
        // }
        // catch (tf::TransformException ex){
        //     NODELET_WARN("%s",ex.what());
        // }
        // transformationMatrix = tftransform2rotm(ins2Velo);
        transformationMatrix << 0.999501,0.0304109,0.00848289,-2.08752,
                                -0.0303736,    0.999529, -0.00448464,   0.0823302,
                                -0.00861528,  0.00422475,    0.999954,    -1.45733,
                                0,           0,           0,           1;
        odom_bicycle = rotm2odometry( odom2rotm(odom_bicycle)*transformationMatrix,stamp,frame_id,child_frame_id);
    }
    odom_bicyclebs_pub.publish(odom_bicycle);
    //std::cout<<"速度"<< v_ <<std::endl;
    //std::cout<<"角速度"<< w_ <<std::endl;
    //std::cout<< std::setprecision(15)<<"rostimepub:"<<rostimepub<<std::endl;
    //std::cout<< std::setprecision(15)<<"deltapub:"<<deltapub<<std::endl;
    //std::cout<< std::setprecision(15)<<"pubtime:"<<pubtime<<std::endl;

   geometry_msgs::TransformStamped OdomBasefootprintTransMsg_;
   OdomBasefootprintTransMsg_.header.stamp = ros::Time::now();
   OdomBasefootprintTransMsg_.header.frame_id = frame_id;
   OdomBasefootprintTransMsg_.child_frame_id = child_frame_id;

   OdomBasefootprintTransMsg_.transform.translation.x = odom_bicycle.pose.pose.position.x;
   OdomBasefootprintTransMsg_.transform.translation.y = odom_bicycle.pose.pose.position.y;
   OdomBasefootprintTransMsg_.transform.translation.z = 0.0;
   OdomBasefootprintTransMsg_.transform.rotation = odom_bicycle.pose.pose.orientation;
   br->sendTransform(OdomBasefootprintTransMsg_);

    output_count_ = output_count_ + 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bicycleslamlidar");
    ros::NodeHandle node;
    ros::NodeHandle p_nh("~");

    cov_slampos = p_nh.param<double>("cov_slampos", 0.04);
    cov_slamyaw = p_nh.param<double>("cov_slamyaw", 0.01);
    cov_slamvel = p_nh.param<double>("cov_slamvel", 0.1);
    cov_imuacc = p_nh.param<double>("cov_imuacc", 0.1);
    cov_imuang = p_nh.param<double>("cov_imuang", 0.1);
    cov_swang = p_nh.param<double>("cov_swang", 0.1);
    lr = p_nh.param<double>("lr", 1.3);
    lf = 2.85 - lr;
    cout<<"lr:"<<lr<<endl;
    cout<<"lf:"<<lf<<endl;
    pbeta = p_nh.param<double>("pbeta", 0.1);
    pa = p_nh.param<double>("pa", 0.5);
    K = p_nh.param<double>("K", 14);
    
    flag_linearacc = p_nh.param<bool>("flag_linearacc", false);
    flag_slamv = p_nh.param<bool>("flag_slamv", false);
    flag_cali_imu = p_nh.param<bool>("flag_cali_imu", false);
    flag_slam_insout = p_nh.param<bool>("flag_slam_insout", false);
    frame_id = p_nh.param<string>("frame_id", "odom");
    child_frame_id = p_nh.param<string>("child_frame_id", "base_foot");

    g_filter_bicycle.setModelParameter(BICYCLEModelParameter(pbeta, pa, lr, lf)); //sigma2_beta, sigma2_a, l

    odom_bicyclebs_pub = node.advertise<nav_msgs::Odometry>("/relocalization/bicycle", 10000);//pub出估计的结果
    br.reset(new tf2_ros::TransformBroadcaster());
    // tr.reset(new tf2_ros::transformListener());

    ros::Subscriber SLAMPOS_sub = node.subscribe("/odometry", 10000, SlamCallback);
    ros::Subscriber IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 10, imuCallback);
    ros::Subscriber SWHANG_sub = node.subscribe("/vehicle/steering_report", 10000,swhangCallback);//转角
    ros::Subscriber LIDAR_sub = node.subscribe("/lidar_odom", 10000, LidarCallback);
    ros::Subscriber LIDARSIG_sub = node.subscribe("/lidar_stamp", 10000, LidarsigCallback);
    ros::Subscriber WHEEL_sub = node.subscribe("/vehicle/wheel_speed_report", 10000,wheelCallback);//轮速，100hz

    ros::spin();//循环
}//看闭合结果
