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
#include "ekf_core/models/akerman.hpp"
#include "ekf_core/extended_kalman_filter.hpp"
#include <transform_utility.hpp>

using namespace std;
using namespace chrono;//计时用的
using namespace Eigen;

double w_limit=0.05;

struct Zpre_slam
{
    double x, y, t;//获取时的预测,速度，角速度，位置，朝向
    double stamp;//时间，为了验证
};
queue<Zpre_slam> M_slam;
Zpre_slam n_slam;

// std::unique_ptr<tf2_ros::TransformBroadcaster> br;

boost::mutex inputmutex;/////////////////////////////////////////////////////////////////////////////////////////////////、、、、、、、、、、

Eigen::IOFormat eigen_csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, "," , "," , "" , "" , "", "");

class AkermanEKF
{
public:
    AkermanEKF()
    {
        ros::NodeHandle node;
        ros::NodeHandle p_nh("~");

        odom_akermanbs_pub = node.advertise<nav_msgs::Odometry>("/relocalization/akermanekf", 10000);//pub出估计的结果

        SLAMPOS_sub = node.subscribe("/odometry", 10000, &AkermanEKF::SlamCallback, this);
        SLAMSIG_sub = node.subscribe("/stamp", 10000, &AkermanEKF::SlamsigCallback, this);

        IMU_sub = node.subscribe("/pioneer_sensors/EKF_Localization_RS232/raw_acceleration", 10, &AkermanEKF::imuCallback, this);//200hz
        // SWHANG_sub = node.subscribe("/vehicle/steering_report", 10000, &AkermanEKF::swhangCallback, this);//转角 100hz
        WHEEL_sub = node.subscribe("/vehicle/wheel_speed_report", 10000, &AkermanEKF::wheelCallback, this);//轮速，100hz
        
    
        cov_slampos = p_nh.param<double>("cov_slampos", 0.04);
        cov_slamyaw = p_nh.param<double>("cov_slamyaw", 0.01);
        cov_slamvel = p_nh.param<double>("cov_slamvel", 0.1);
        cov_imuacc = p_nh.param<double>("cov_imuacc", 0.1);
        cov_imuang = p_nh.param<double>("cov_imuang", 0.1);
        cov_swang = p_nh.param<double>("cov_swang", 0.1);
        cov_wheel = p_nh.param<double>("cov_wheel", 0.1);

        pbeta = p_nh.param<double>("pbeta", 0.1);
        pa = p_nh.param<double>("pa", 0.5);
        K = p_nh.param<double>("K", 14);
        per = p_nh.param<double>("per", 14);
        
        use_swhang = p_nh.param<bool>("use_swhang", false);

        flag_linearacc = p_nh.param<bool>("flag_linearacc", false);
        flag_slamv = p_nh.param<bool>("flag_slamv", false);
        flag_cali_imu = p_nh.param<bool>("flag_cali_imu", false);
        frame_id = p_nh.param<string>("frame_id", "odom");
        child_frame_id = p_nh.param<string>("child_frame_id", "base_foot");

        g_filter_akerman.setModelParameter(AKERMANModelParameter(pbeta,pa)); //sigma2_beta, sigma2_a, l
    }
    void SlamsigCallback(const nav_msgs::OdometryConstPtr& input)//记录状态信息
    {
        if(!inite)
        {
            return;
        }
        double t2_ = input->header.stamp.toSec();//bag时间戳,存储
        cout << "slam_signal" << endl;

        int total_length_akerman = 0;//先统计这次更新用多少维
        total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<AccelerationMeasurement>();
        total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<AngularvelocityMeasurement>();
        if(g_slam_measurement_changed)
        {
            total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<PositionMeasurement>();
            total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<HeadingMeasurement>();
        } 
        if(g_sw_angle_measurement_changed) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<SteeringwheelangleMeasurement>();
        if(g_wheel_velocity_measurement_changed) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<WheelMeasurement>();

        bool f = true;
        g_filter_akerman.beginAddMeasurement(t2_, total_length_akerman, f);

        Zpre_slam Zs;
        Zs.x = g_filter_akerman.state_pre_(0);//prediction;
        Zs.y = g_filter_akerman.state_pre_(1);
        Zs.t = g_filter_akerman.state_pre_(2);
        Zs.stamp = t2_;//获取时间

        M_slam.push(Zs); 
        cout<<"**M_slam"<< M_slam.size() <<"**"<<endl;
    }

    //保证ｓｋ对齐机制，不要漏帧错帧　　附带当时的时间戳
    void SlamCallback(const nav_msgs::OdometryConstPtr& input)//两个时间戳，bag原始的时间戳以及计算时间长度，得到解算完的对应bag的时间
    {
        double timestamp = ros::Time::now().toSec();//处理后的时间
        timestamp_old = input->header.stamp.toSec();//点云获取时间
        inite = true;
        N_ += 1;

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

        if(N_ == 0)
        {
            g_filter_akerman.initnew(timestamp, input->pose.pose.position.x, input->pose.pose.position.y, yaw);
            inite_g_slam_measurement = true;//GPS测量
            N_ += 1;
        }
        g_slam_measurement_changed = true;
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
            cout << setprecision(15) << "SLAM后一个:" << n_new.stamp << endl;
            cout << setprecision(15) << "SLAM栈顶" << n.stamp <<endl;
            cout << setprecision(15) << "SLAM输入" << timestamp_old <<endl;
            while(n.stamp != timestamp_old)//可能出现sig在结果之后吗？序列顺序？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
            {
                cout << setprecision(15) << "SLAM不对齐了ｎ（＊≧▽≦＊）ｎ" <<endl;
                cout << setprecision(15) << "SLAM栈顶" << n.stamp << endl;
                cout << setprecision(15) << "SLAM输入" << timestamp_old << endl;
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
            if(!isnan(a))//判断是否是nan
            {
                Matrix<double, 2, 1> measurement;
                double X_ = input->pose.pose.position.x;
                double Y_ = input->pose.pose.position.y;
                z_ = input->pose.pose.position.z;
                measurement << X_ - n.x, Y_ - n.y;            
                Matrix<double, 2, 2> covariance = MatrixXd::Identity(2,2) * cov_slampos;

            //     covariance << input->pose.covariance[0] * input->pose.covariance[0], 0,
            // 0,input->pose.covariance[1] * input->pose.covariance[1];/////////////////////////////////////////////////含义？？？？
                g_slam_position_measurement.setMeasurement(measurement, covariance, timestamp);
                measurement1_slam = measurement;

                Matrix<double, 1, 1> measurement1;
                measurement1 << yaw - n.t;
                Matrix<double, 1, 1> covariance1;
                covariance1 << cov_slamyaw;
                g_slam_heading_measurement.setMeasurement(measurement1, covariance1, timestamp);
                measurement2_slam = measurement1;
            }
        }
    }
    // ////****ADD****////
    void swhangCallback(const dbw_mkz_msgs::SteeringReportConstPtr& input)
    {
        //boost::mutex::scoped_lock lock(inputmutex);
        double timestamp = input->header.stamp.toSec();

        if(!inite || !use_swhang)
        {
            return;
        }
        double a = input->steering_wheel_angle;
        if(!isnan(a))
        {
            inite_g_sw_angle_measurement=true;

            Matrix<double, 1, 1> measurement;
            measurement << input->steering_wheel_angle / K;//K是转角比例
            Matrix<double, 1, 1> swanglecovariance;
            swanglecovariance << cov_swang;
            g_sw_angle_measurement.setMeasurement(measurement, swanglecovariance, timestamp);//写入成员变量

            g_sw_angle_measurement_changed = true;
            cout<<"VEHangle:"<< measurement << endl;
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
            if(!isnan(a))
            {
                if(input->front_left == 0 && input->front_right == 0 && input->rear_left == 0 && input->rear_right == 0)
                {
                    flag_zero = true;
                }
                inite_g_wheel_velocity_measurement=true;
                Matrix<double, 1, 1> wheelcovariance;
                wheelcovariance << MatrixXd::Identity(1,1) * cov_wheel;

                Matrix<double, 1, 1> measurement1;
                Matrix<double, 1, 1> measurement2;
                measurement1 << 0.5* (input->rear_left + input->rear_right) * g_filter_akerman.model_parameter_.R_w;
                measurement2 << (-input->rear_left + input->rear_right) * g_filter_akerman.model_parameter_.R_w / g_filter_akerman.model_parameter_.W;
                measurement1 *= per;
                measurement2 *= per;
                g_wheel_velocity_measurement.setMeasurement(measurement1, wheelcovariance, timestamp);//写入成员变量
                g_wheel_angularvelocity_measurement.setMeasurement(measurement2, wheelcovariance/10, timestamp);//写入成员变量
                g_wheel_velocity_measurement_changed = true;
            }
        }
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& input)
    {
        cout << cov_slampos << endl;
        rostimebegin = ros::Time::now().toSec();///////////////////////////////////////////////////////////////////////////////////////////////时间对齐
        //cout<< setprecision(15)<<"rostimebegin:"<<rostimebegin<< endl;

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
                Matrix<double, 1, 1> angularvelocitymeasurement;
                angularvelocitymeasurement << input->angular_velocity.z;
                Matrix<double, 1, 1> angularvelocitycovariance;
                angularvelocitycovariance << cov_imuang;
                g_imu_angularvelocity_measurement.setMeasurement(angularvelocitymeasurement, angularvelocitycovariance, timestamp);
                angular_z_=input -> angular_velocity.z;

                Matrix<double, 1, 1> accelerationmeasurement;
                accelerationmeasurement << input->linear_acceleration.x;
                // if(flag_linearacc)
                // {
                //     Matrix<double, 1, 1> V;
                //     V << outputstate_akerman_(4);
                //     accelerationmeasurement += angularvelocitymeasurement(0) * V;
                // }
                // if (flag_cali_imu)  
                // {
                //     Matrix<double, 2, 2> C2;
                //     C2 << 0, -1,
                //           -1, 0;
                //     accelerationmeasurement = C2 * accelerationmeasurement;
                // }
                Matrix<double, 1, 1> accelerationcovariance;
                accelerationcovariance << cov_imuacc;
                g_imu_acceleration_measurement.setMeasurement(accelerationmeasurement, accelerationcovariance, timestamp);
                // cout << "变换后x向加速度：" << accelerationmeasurement << endl;
                // cout << "另一种x向加速度：" << acc(0) << endl;
            }
        }
        if(imuinite&&inite_g_slam_measurement)
        //if(imuinite)//执行条件
        {
            update(timestamp,stamp);
        }

        rostimeend = ros::Time::now().toSec();
        //cout<< setprecision(15)<<"rostimeend:"<<rostimeend<< endl;
    }

    void update(double timestamp, ros::Time stamp)//来什么量测，用什么来更新
    {
        bool use_slam = g_slam_measurement_changed;
        bool use_swangle = g_sw_angle_measurement_changed;
        bool use_whevel = g_wheel_velocity_measurement_changed;

        {
            boost::mutex::scoped_lock lock(inputmutex);////////////////////////////////////////////////////////////////////////////////////////////////////
    //akerman

            int total_length_akerman = 0;//先统计这次更新用多少维
            total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<AccelerationMeasurement>();
            total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<AngularvelocityMeasurement>();
            if(use_slam) 
            {
                total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<PositionMeasurement>();
                if(flag_slamv) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<VelocityMeasurement>();
                total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<HeadingMeasurement>();
            }
            if(use_swangle) total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<SteeringwheelangleMeasurement>();
            if(use_whevel) 
            {
                total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<AngularvelocityMeasurement>();
                total_length_akerman += decltype(g_filter_akerman)::getLengthForMeasurement<VelocityMeasurement>();
            }

            bool f = false;
            g_filter_akerman.beginAddMeasurement(timestamp, total_length_akerman, f);
            
            g_filter_akerman.AddMeasurement(g_imu_angularvelocity_measurement);
            g_filter_akerman.AddMeasurement(g_imu_acceleration_measurement);
            if(use_slam) 
            {
                Matrix<double, 2, 1> measurement1;
                Matrix<double, 1, 1> measurement2;

                measurement1 << g_filter_akerman.state_pre_(0), g_filter_akerman.state_pre_(1);
                g_slam_position_measurement.changeMeasurement(measurement1);
                g_filter_akerman.AddMeasurement(g_slam_position_measurement);
                measurement1_slam += measurement1;
                cout<<"SLAMpos:"<< measurement1_slam << endl;

                measurement2 << g_filter_akerman.state_pre_(2);
                g_slam_heading_measurement.changeMeasurement(measurement2);
                g_filter_akerman.AddMeasurement(g_slam_heading_measurement);
                measurement2_slam += measurement2;
                cout<<"SLAMheading:"<< measurement2_slam << endl;
            }
            if(use_swangle) g_filter_akerman.AddMeasurement(g_sw_angle_measurement);
            if(use_whevel) 
            {
                g_filter_akerman.AddMeasurement(g_wheel_velocity_measurement);
                g_filter_akerman.AddMeasurement(g_wheel_angularvelocity_measurement);
            }

            if(use_slam) g_slam_measurement_changed = false;
            if(use_swangle) g_sw_angle_measurement_changed = false;
            if(use_whevel) g_wheel_velocity_measurement_changed = false;
        }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////covarience????????????????

        decltype(g_filter_akerman)::Model::FilterVector outputstate_akerman;
        decltype(g_filter_akerman)::Model::FilterMatrix outputcovariance_akerman;
        double outputtime_akerman;

        g_filter_akerman.endAddMeasurement(outputstate_akerman, outputcovariance_akerman, outputtime_akerman, flag_zero);

        // cout << "P = "<< outputcovariance_akerman <<endl;
        cout << "状态 = "<< outputstate_akerman <<endl;
        // cout << "Q = "<< g_filter_akerman.Q_ <<endl;
        // cout << "W = "<< g_filter_akerman.W_ <<endl;

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
        odom_akerman.header.stamp = stamp;
        odom_akerman.header.frame_id = frame_id;
        odom_akerman.child_frame_id = child_frame_id;

        odom_akerman.pose.pose.position.x = position_akerman(0);
        odom_akerman.pose.pose.position.y = position_akerman(1);
        odom_akerman.pose.pose.position.z = z_;

        tf2::Quaternion quat_akerman;
        quat_akerman.setRPY(0, 0, heading_akerman(0));
        odom_akerman.pose.pose.orientation.x = quat_akerman.x();
        odom_akerman.pose.pose.orientation.y = quat_akerman.y();
        odom_akerman.pose.pose.orientation.z = quat_akerman.z();
        odom_akerman.pose.pose.orientation.w = quat_akerman.w();
        odom_akerman.twist.twist.linear.x = velocity_akerman(0) * cos(heading_akerman(0));
        odom_akerman.twist.twist.linear.y = velocity_akerman(0) * sin(heading_akerman(0));
        odom_akerman.twist.twist.linear.z = 0.0;
        odom_akerman.twist.twist.angular.x = 0.0;
        odom_akerman.twist.twist.angular.y = 0.0;
        odom_akerman.twist.twist.angular.z = angularvelocity_akerman(0);
        v_ = outputstate_akerman(3);
        w_ = outputstate_akerman(4);

        odom_akermanbs_pub.publish(odom_akerman);
        //cout<< setprecision(15)<<"rostimepub:"<<rostimepub<<endl;
        //cout<< setprecision(15)<<"deltapub:"<<deltapub<<endl;
        //cout<< setprecision(15)<<"pubtime:"<<pubtime<<endl;

        geometry_msgs::TransformStamped OdomBasefootprintTransMsg_;
        OdomBasefootprintTransMsg_.header.stamp = ros::Time::now();
        OdomBasefootprintTransMsg_.header.frame_id = frame_id;
        OdomBasefootprintTransMsg_.child_frame_id = child_frame_id;

        OdomBasefootprintTransMsg_.transform.translation.x = odom_akerman.pose.pose.position.x;
        OdomBasefootprintTransMsg_.transform.translation.y = odom_akerman.pose.pose.position.y;
        OdomBasefootprintTransMsg_.transform.translation.z = 0.0;
        OdomBasefootprintTransMsg_.transform.rotation = odom_akerman.pose.pose.orientation;
        br.sendTransform(OdomBasefootprintTransMsg_);

        output_count_ = output_count_ + 1;
    }
private:
    

    ros::Publisher odom_akermanbs_pub;

    ros::Subscriber SLAMPOS_sub;
    ros::Subscriber SLAMSIG_sub;
    ros::Subscriber IMU_sub;
    ros::Subscriber WHEEL_sub;
    ros::Subscriber SWHANG_sub;

    tf2_ros::TransformBroadcaster br;

    ExtendedKalmanFilter<AKERMANModel> g_filter_akerman;
    decltype(g_filter_akerman)::Model::FilterVector outputstate_akerman_;//实时输出
    double cov_slampos, cov_imuacc, cov_imuang, cov_slamyaw, cov_slamvel, cov_swang, cov_wheel;
    string frame_id, child_frame_id;
    double pbeta, pa;
    bool flag_slamv, flag_linearacc, flag_cali_imu;
    bool use_swhang;
    double K, per;//方向盘转角比车轮转角,测量轮角速度到轮速的误差比例，半径误差
    
    PositionMeasurement g_slam_position_measurement;//在measurements里定义的类
    VelocityMeasurement g_wheel_velocity_measurement;//4维
    AngularvelocityMeasurement g_wheel_angularvelocity_measurement;
    HeadingMeasurement g_slam_heading_measurement;
    AngularvelocityMeasurement g_imu_angularvelocity_measurement;
    AccelerationMeasurement g_imu_acceleration_measurement;
    SteeringwheelangleMeasurement g_sw_angle_measurement;//方向盘转角
    VelocityMeasurement g_slam_velocity_measurement;

    bool g_slam_measurement_changed;
    bool g_sw_angle_measurement_changed;
    bool g_wheel_velocity_measurement_changed;

    bool inite_g_slam_measurement = false;
    bool inite_g_sw_angle_measurement = false;
    bool inite_g_wheel_velocity_measurement = false;

    bool inite = false;
    bool flag_zero = false;

    double delta_;
    double timestamp_old;
    Matrix<double, 2, 1> measurement1_slam;//pos
    Matrix<double, 1, 1> measurement2_slam;//yaw
    double rostimebegin;
    double rostimeend;
    double rostimepub;

    double z_=0;
    double angular_z_=0;
    int output_count_ = 0;
    int N_ = -1;
    double v_=0.02;
    double w_=0.01;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "akermangps");
    AkermanEKF akerman;
    // br.reset(new tf2_ros::TransformBroadcaster());
    ros::spin();//循环
}//看闭合结果