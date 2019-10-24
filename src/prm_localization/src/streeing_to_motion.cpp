//
// Created by vickylzy on 19-9-14.
//


#include <ros/ros.h>
#include <ros/timer.h>
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// ros_msg
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/WheelSpeedReport.h>
#include <nav_msgs/Odometry.h>
//eigen
#include <Eigen/Dense>
//tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
//cpp
#include <ctime>
#include <limits.h>
#include <mutex>
#include <math.h>
#include <boost/circular_buffer.hpp>
//untility
#include <prm_localization/transform_utility.hpp>

const float car_Centerlength = 2.8498f ;
const float stree_wheel_times = 14.80f;
const float wheel_radius = 0.356f;
typedef message_filters::sync_policies::ApproximateTime<dbw_mkz_msgs::SteeringReport,dbw_mkz_msgs::WheelSpeedReport> pioneer_msg_sync_policy;

using namespace std;

class StreeingCompute {

public:
    StreeingCompute(){

    }
    virtual  ~StreeingCompute(){
    }

    void onInit()  {
        //suber and puber
        streeing_motion_puber = nh.advertise<nav_msgs::Odometry>("/localization/streeing_motion",10);
        steering_sub = new message_filters::Subscriber<dbw_mkz_msgs::SteeringReport>(nh,"/vehicle/steering_report",5);
        wheel_sub  = new message_filters::Subscriber<dbw_mkz_msgs::WheelSpeedReport> (nh,"/vehicle/wheel_speed_report",5);
        sync = new message_filters::Synchronizer<pioneer_msg_sync_policy> (pioneer_msg_sync_policy(10), *steering_sub,*wheel_sub);
        sync->registerCallback(boost::bind(&StreeingCompute::stree_wheel_callback,this,_1,_2));
    }

public:
    void stree_wheel_callback(const dbw_mkz_msgs::SteeringReportConstPtr& steering_msg, const dbw_mkz_msgs::WheelSpeedReportConstPtr& wheel_msg){
//        cout<<"sync_success_onece"<<endl;
        if (!first_steering_flag){
            first_steering_flag= true;
            last_time_tag = steering_msg->header.stamp;
        } else {
            Matrix4f this_motion = transform_steer_to_motion(steering_msg,wheel_msg,last_time_tag);
            nav_msgs::Odometry frame_odom = rotm2odometry(this_motion,last_time_tag,"","");
            streeing_motion_puber.publish(frame_odom);
            last_time_tag = steering_msg->header.stamp;
        }
    }



    Eigen::Matrix4f transform_steer_to_motion (const dbw_mkz_msgs::SteeringReportConstPtr& steering_msg,const dbw_mkz_msgs::WheelSpeedReportConstPtr& wheel_msg, const ros::Time lastTag) const {

        Matrix4f curr_motion;
        curr_motion.setIdentity();
        float x_mov = 0;
        float y_mov =  0;
        float back_center_velocity = (wheel_msg->rear_left+wheel_msg->rear_right)/2 * wheel_radius;
        // time_to_former_msg
        ros::Duration time_collaspe = steering_msg->header.stamp-lastTag;
        // forward_no_turn
        if(steering_msg->steering_wheel_angle==0){
            x_mov =  back_center_velocity * time_collaspe.toSec();
            y_mov =  0;
            curr_motion(0,3) = x_mov;
            curr_motion(1,3) = y_mov;
        } else{
            // front_wheel_turn_in_rad
            float car_front_theta = steering_msg->steering_wheel_angle / stree_wheel_times;

            // back_turn_radius
            float turn_radius= cos(car_front_theta) / sin(car_front_theta) * car_Centerlength;
            // back_wheel_center_turn_in_rad
            float theta = back_center_velocity / turn_radius * time_collaspe.toSec();
            x_mov =  turn_radius * sin(theta);
            y_mov =  turn_radius * (1 - cos(theta));
            curr_motion.block(0,0,3,3) = euler2rot(0,0,theta);
            curr_motion(0,3) = x_mov;
            curr_motion(1,3) = y_mov;
        }
        return curr_motion;
    }

private:
    //ros
    ros::NodeHandle nh;
    //suber and puber
    ros::Publisher streeing_motion_puber;
    message_filters::Subscriber<dbw_mkz_msgs::SteeringReport> *steering_sub;
    message_filters::Subscriber<dbw_mkz_msgs::WheelSpeedReport> *wheel_sub;
    message_filters::Synchronizer<pioneer_msg_sync_policy> *sync;
    //parameter
    ros::Time last_time_tag;

    //flag
    bool first_steering_flag= false;
};

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "streeing_to_motion");
    StreeingCompute streeingCompute;
    streeingCompute.onInit();
    ros::spin();

    return 0;
}