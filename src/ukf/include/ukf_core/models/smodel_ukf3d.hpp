#ifndef SMODELS_HPP
#define SMODELS_HPP

#include "ukf_core/kalman_filter_model_base.hpp"
#include "ukf_core/unscented_kalman_filter.hpp"
#define PI           3.14159265358979323846  /* pi */
using Eigen::MatrixXd;
using Eigen::VectorXd;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Quaternion<double> Quaterniond;
struct SModelParameter : KalmanFilterModelParameterBase
{
    SModelParameter() //构造函数
    {
        this->process_noise = Eigen::MatrixXd::Identity(22, 22);
        this->process_noise.middleRows(0, 3) *= 10.0;//pos
        this->process_noise.middleRows(3, 3) *= 10.0;//v
        this->process_noise.middleRows(6, 4) *= 10.0;//q
        this->process_noise.middleRows(10, 3) *= 10.0;//a
        this->process_noise.middleRows(13, 3) *= 10.0;//w
        this->process_noise.middleRows(16, 3) *= 1e-3;//a_bias
        this->process_noise.middleRows(19, 3) *= 1e-3;//g_bias
    }
     //重载
    SModelParameter(MatrixXd process_noise_in)
    {
        this->process_noise = process_noise_in;
    }

    Eigen::MatrixXd process_noise;
};

class SModel : public KalmanFilterModelBase<22, SModelParameter>
{

public:
    
    SModel()
    {
        num = 22;
    }

    static void initState(
        const ModelParameter& parameter,
        FilterVector& xsig,
        MatrixXd& xsig_covariance
    )
    {
        xsig.setZero(22, 1);
        xsig(6) = 1.0;
        xsig_covariance.setIdentity(22,22);
        xsig_covariance *= 99;

    }

    static void updatePrediction(
        const ModelParameter& parameter,
        const FilterVector xsig,
        const double delta,
        FilterVector& xsig_prediction
    )//属于类的函数
    {
        Vector3d p = xsig.middleRows(0, 3);
        Vector3d v = xsig.middleRows(3, 3);

        Vector4d qq;
        qq << xsig[6], xsig[7], xsig[8], xsig[9];

        Vector3d raw_acc = xsig.middleRows(10, 3);
        //std::cout << "raw_acc:" << raw_acc << std::endl; 
        Vector3d raw_gyro = xsig.middleRows(13, 3);

        Vector3d acc_bias = xsig.middleRows(16, 3);
        Vector3d gyro_bias = xsig.middleRows(19, 3);

        // position
        xsig_prediction.middleRows(0, 3) = p + v * delta;//零阶保持

        // velocity
        Eigen::Quaternion<double> q(qq);
        q.normalize();
        Eigen::Matrix<double, 3, 3> Cbn = q.matrix();
        std::cout << "姿态：" << Cbn << std::endl;
        Vector3d g(0.0f, 0.0f, -9.80665f);
        Vector3d acc_ = raw_acc - acc_bias;
        Vector3d acc = Cbn * acc_;//旋转
        std::cout << "rawacc:" << raw_acc << std::endl;
        std::cout << "acc:" << acc - g << std::endl;
        xsig_prediction.middleRows(3, 3) = v + (acc - g) * delta;		// acceleration didn't contribute to accuracy due to large noise
        //std::cout << "acc:" << acc - g << std::endl; 
        //std::cout << "delta:" << delta << std::endl;        
        //std::cout << "v:" << v + (acc - g) * delta << std::endl;
        // orientation
        Vector3d gyro = raw_gyro - gyro_bias;
        Quaterniond dq(1, gyro[0] * delta / 2, gyro[1] * delta / 2, gyro[2] * delta / 2);
        dq.normalize();
        Quaterniond q_ = (q * dq).normalized();
        // Eigen::Matrix<double, 4, 4> DQ;
        // DQ << 2, -gyro(0)*delta, -gyro(1)*delta, -gyro(2)*delta,
        //       gyro(0)*delta, 2, gyro(2)*delta, -gyro(1)*delta,
        //       gyro(1)*delta, -gyro(2)*delta, 2, gyro(0)*delta,
        //       gyro(2)*delta, gyro(1)*delta, -gyro(0)*delta, 2;
        // DQ = DQ/2;
        // Vector4d qq_;
        // qq_ = DQ * qq;
        // Eigen::Quaternion<double> q_(qq_);
        // q_.normalize();
        xsig_prediction.middleRows(6, 4) << q_.w(), q_.x(), q_.y(), q_.z();

        xsig_prediction.middleRows(10, 3) = xsig.middleRows(10, 3);		// constant acceleration
        xsig_prediction.middleRows(13, 3) = xsig.middleRows(13, 3);		// constant angular velocity
        xsig_prediction.middleRows(16, 3) = xsig.middleRows(16, 3);		// constant bias on acceleration
        xsig_prediction.middleRows(19, 3) = xsig.middleRows(19, 3);		// constant bias on angular velocity
    }
public:
    int num;
    
};
#endif