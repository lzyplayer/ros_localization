

#ifndef EIGEN_STAMPED
#define EIGEN_STAMPED

//eigen
#include <Eigen/Dense>
#include <ros/ros.h>
#include <utility>

namespace Eigen_stamped_class{
    class Eigen_stamped{
        private:
    public:
        Eigen_stamped(Matrix4f motion, const ros::Time &stamp) : motion(std::move(motion)), stamp(stamp) {}


        const Matrix4f &getMotion() const {
            return motion;
        }

        const ros::Time &getStamp() const {
            return stamp;
        }

    private:
        Eigen::Matrix4f motion;
        ros::Time stamp;
    };
}

#endif //EIGEN_STAMPED