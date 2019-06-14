#include "ros/ros.h"
#include "std_msgs/String.h"
#include <prm_localization/fdcp_register.hpp>
//user test
#include <vector>
#include <iostream>

using namespace std;
class PRM_localization{
public:
    PRM_localization(){

    }
    virtual ~PRM_localization(){
    }

private:
    ros::NodeHandle curr_node;
    ros::Publisher curr_pub;
    ros::Subscriber curr_sub;
//    ros::Rate *loop_rate = new ros::Rate(5);


};



//
int main(int argc, char *argv[])
{
    vector<int>array(9);
    array.push_back(2);
    array.push_back(2);
    array.push_back(2);
    array.push_back(2);
    array.clear();
    cout<<array.size();
    array.push_back(5);
    array.push_back(3);
    cout<<array.size();
//    ros::init(argc, argv, "register");
//
//    PRM_localization prm_localization;
//
//    ros::spin();
//
    return 0;
}