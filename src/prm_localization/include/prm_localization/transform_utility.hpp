
#ifndef TRANSFORM_UTILITY
#define TRANSFORM_UTILITY

#include <Eigen/Dense>
#include <ros/ros.h>

/**function for conversation**/

using namespace Eigen;

Matrix3f euler2rot(const float x_pi,const float y_pi,const float z_pi){
    Matrix3f m;
    m = AngleAxisf(x_pi*M_PI, Vector3f::UnitX())
        * AngleAxisf(y_pi*M_PI, Vector3f::UnitY())
        * AngleAxisf(z_pi*M_PI, Vector3f::UnitZ());
    return m;
}
Vector3f rot2euler(const Matrix3f& m){
    return m.eulerAngles(0,1,2);
}


/**
 *
 * @brief convert a Eigen::Matrix to TransformedStamped
 * @param stamp           timestamp
 * @param pose            pose matrix
 * @param frame_id        frame_id
 * @param child_frame_id  child_frame_id
 * @return transform
 */
geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3,3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
}

#endif //TRANSFORM_UTILITY
