//2d
#ifndef SMODELS_HPP
#define SMODELS_HPP

#include "ukf_core/kalman_filter_model_base.hpp"
#include "ukf_core/unscented_kalman_filter.hpp"
#define PI           3.14159265358979323846  /* pi */
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 1, 1> Vectord;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

struct SModelParameter : KalmanFilterModelParameterBase
{
    SModelParameter() //构造函数
    {
        this->process_noise = Eigen::MatrixXd::Identity(11, 11);
        this->process_noise.middleRows(0, 2) *= 0.1;//pos
        this->process_noise.middleRows(2, 2) *= 0.1;//v
        this->process_noise.middleRows(4, 1) *= 0.005;//yaw
        this->process_noise.middleRows(5, 2) *= 1;//a
        this->process_noise.middleRows(7, 1) *= 0.01;//w
        this->process_noise.middleRows(8, 2) *= 1e-6;//ab
        this->process_noise.middleRows(10, 1) *= 1e-6;//wb
    }
     //重载
    SModelParameter(MatrixXd process_noise_in)
    {
        this->process_noise = process_noise_in;
    }

    Eigen::MatrixXd process_noise;
};

class SModel : public KalmanFilterModelBase<11, SModelParameter>
{

public:
    
    SModel()
    {
        num = 11;
    }

    static void initState(
        const ModelParameter& parameter,
        FilterVector& xsig,
        MatrixXd& xsig_covariance
    )
    {
        xsig.setZero(11, 1);
        xsig_covariance.setIdentity(11,11);
        xsig_covariance *= 0.1;
    }


    static void updatePrediction(
        const ModelParameter& parameter,
        const FilterVector xsig,
        const double delta,
        FilterVector& xsig_prediction
    )//属于类的函数
    {
        // std::cout << "fine1" << std::endl;
        Vector2d p = xsig.middleRows(0, 2);
        Vector2d v = xsig.middleRows(2, 2);
        double yaw = xsig(4);
        Vector2d raw_acc = xsig.middleRows(5, 2);//状态，对应的是IMU系下的加速度
        //std::cout << "raw_acc:" << raw_acc << std::endl; 
        double raw_w = xsig(7);

        Vector2d acc_bias = xsig.middleRows(8, 2);
        double w_bias = xsig(10);
        Vector2d acc_ = raw_acc - acc_bias;
        double w = raw_w - w_bias;

        double ax = acc_(0);
        double ay = acc_(1);
        // double ax = 0;
        // double ay = 0;
        double vx = v(0);
        double vy = v(1);
        double a1;
        double a2;
        double LIMITW = 0.001;
        double delta2 = delta * delta;
        double yawt = yaw + w * delta;
        //pos
        cout << "w" << w << endl;
        if(abs(w) > LIMITW)
        {
            a1 = vy*(cos(yawt)-cos(yaw)) + vx*(sin(yawt)-sin(yaw));
            a2 = -vx*(cos(yawt)-cos(yaw)) + vy*(sin(yawt)-sin(yaw));
            xsig_prediction(0) = p(0) + (1/w) * a1 + (1/w)*(1/w) * (ax*(cos(yawt)-cos(yaw)) - ay*(sin(yawt)-sin(yaw)) + ay*delta*w*cos(yawt) + ax*delta*w*sin(yawt));
            xsig_prediction(1) = p(1) + (1/w) * a2 + (1/w)*(1/w) * (ay*(cos(yawt)-cos(yaw)) + ax*(sin(yawt)-sin(yaw)) + ay*delta*w*sin(yawt) - ax*delta*w*cos(yawt));
        }
        else
        {
            a1 = -vy * sin(yaw) * delta + vx * cos(yaw) * delta;
            a2 = vx * sin(yaw) * delta + vy * cos(yaw) * delta;
            xsig_prediction(0) = p(0) + a1 + (1/2.0) * delta2 * (ax*cos(yaw) - ay*sin(yaw));
            xsig_prediction(1) = p(1) + a2 + (1/2.0) * delta2 * (ay*cos(yaw) + ax*sin(yaw));
        }
        //v yaw
        xsig_prediction.middleRows(2, 2) = v;// + acc_ * delta;
        xsig_prediction(4) = yawt;

        xsig_prediction.middleRows(5, 2) = xsig.middleRows(5, 2);		// constant acceleration
        xsig_prediction.middleRows(7, 1) = xsig.middleRows(7, 1);		// constant angular velocity
        xsig_prediction.middleRows(8, 2) = xsig.middleRows(8, 2);		// constant bias on acceleration
        xsig_prediction.middleRows(10, 1) = xsig.middleRows(10, 1);		// constant bias on angular velocity
    }
public:
    int num;
    
};
#endif