//2d
#ifndef SMODELS_HPP
#define SMODELS_HPP

#include "ukf_core/kalman_filter_model_base.hpp"
#include "ukf_core/unscented_kalman_filter.hpp"
#define PI           3.14159265358979323846  /* pi */
using Eigen::MatrixXd;
using Eigen::VectorXd;
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
        this->process_noise.middleRows(7, 1) *= 0.001;//w
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
        Vector2d p = xsig.middleRows(0, 2);
        Vector2d v = xsig.middleRows(2, 2);
        double yaw = xsig(4);        
        Vector2d raw_acc = xsig.middleRows(5, 2);
        //std::cout << "raw_acc:" << raw_acc << std::endl; 
        double raw_gyro = xsig(7);

        Vector2d acc_bias = xsig.middleRows(8, 2);
        double gyro_bias = xsig(10);

        // velocity
        Vector2d acc_ = raw_acc - acc_bias;
        Eigen::Matrix<double, 2, 2> Cbn;
        Cbn << cos(yaw), -sin(yaw),
               sin(yaw), cos(yaw);
        // xsig_prediction.middleRows(2, 2) = v + acc * delta;	// acceleration didn't contribute to accuracy due to large noise
        xsig_prediction.middleRows(2, 2) = v;
        // Vector2d v_ = Cbn * v;//旋转
        
        // position
        xsig_prediction.middleRows(0, 2) = p + Cbn * (v * delta );//零阶保持
        // orientation
        double gyro = raw_gyro - gyro_bias;
        double yaw_ = yaw + gyro * delta;

        xsig_prediction.middleRows(4, 1) << yaw_;

        xsig_prediction.middleRows(5, 2) = xsig.middleRows(5, 2);		// constant acceleration
        xsig_prediction.middleRows(7, 1) = xsig.middleRows(7, 1);		// constant angular velocity
        xsig_prediction.middleRows(8, 2) = xsig.middleRows(8, 2);		// constant bias on acceleration
        xsig_prediction.middleRows(10, 1) = xsig.middleRows(10, 1);		// constant bias on angular velocity
    }
public:
    int num;
    
};
#endif