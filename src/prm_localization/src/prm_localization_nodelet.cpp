#include "ros/ros.h"
#include "std_msgs/String.h"
#include <prm_localization/fdcp_register.hpp>
//user test
#include <vector>
#include <iostream>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace prm_localization {
    using namespace std;

    class PRMLocalizationNodelet : public nodelet::Nodelet {
    public:
        PRMLocalizationNodelet() {

        }

        virtual ~PRMLocalizationNodelet() {
        }

        void onInit() override {
            nh = getNodeHandle();
            mt_nh = getMTNodeHandle();
            private_nh = getPrivateNodeHandle();
//            initialize_params();
            NODELET_INFO("check On init");
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;

        ros::Publisher curr_pub;
        ros::Subscriber curr_sub;
//    ros::Rate *loop_rate = new ros::Rate(5);


    };


//
//    int main(int argc, char *argv[]) {
//
//        vector<int> array(9);
//        array.push_back(2);
//        array.push_back(2);
//        array.push_back(2);
//        array.push_back(2);
//        array.clear();
//        cout << array.size();
//        array.push_back(5);
//        array.push_back(3);
//        cout << array.size();
//    ros::init(argc, argv, "register");
//
//    PRM_localization prm_localization;
//
//    ros::spin();
//
//        return 0;
//    }
}

PLUGINLIB_EXPORT_CLASS(prm_localization::PRMLocalizationNodelet, nodelet::Nodelet)