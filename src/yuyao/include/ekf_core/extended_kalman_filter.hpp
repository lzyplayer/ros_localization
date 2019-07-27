#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

template<typename TModel>//模型类型，参数是 类 名称
class ExtendedKalmanFilter
{
public:
    typedef TModel Model;
    
    typename TModel::FilterMatrix Q_;
    Eigen::MatrixXd W_;

    void setModelParameter(const typename TModel::ModelParameter& parameter)
    {
        model_parameter_ = parameter;
    }

    void init(double timestamp)
    {
        identity_p_.setIdentity();//单位阵
        TModel::initState(model_parameter_, state_, state_covariance_);
        timestamp_ = timestamp;
    }

    void beginAddMeasurement(double timestamp, int total_length, bool flag)//当前测量的变量数
    {
        if(is_adding_measurement_) throw;

        // resize the matrixes to fit the input匹配大小
        measurement_stack_.resize(total_length);//eigen库函数
        estimated_measurement_stack_.resize(total_length);
        h_matrix_stack_.resize(total_length, TModel::NumberState);
        residual_covariance_.resize(total_length, total_length);
        measurement_covariance_stack_.resize(total_length, total_length);
        measurement_covariance_stack_.setZero();
        measurement_prediction_.resize(total_length);
        measurement_residual_.resize(total_length);
        filter_gain_.resize(TModel::NumberState, total_length);
        angle_flag_stack_.resize(total_length);

        stack_index_ = 0;
        total_length_ = total_length;
        if (flag)
        {
            delta_ = timestamp - timestamp_;//两次测量时差
        }
        else
        {
            delta_ = timestamp - timestamp_;//两次测量时差
            timestamp_ = timestamp;
            is_adding_measurement_ = true;
        }

        // begin cycle
        TModel::updatePrediction(model_parameter_, state_, delta_, state_prediction_, jacobian_, process_noise_);
        state_pre_ = state_prediction_;
        Q_=process_noise_;
        // begin kalman filter core
        // P(k+1|k) = F*P(k|k)*F'+Q
        state_prediction_covariance_ = (jacobian_ * state_covariance_ * jacobian_.transpose())  + process_noise_;//P（K+1|k）
    }

    template<typename TMeasurement>
    typename std::enable_if<!SensorMeasurementConverter::is_support<TMeasurement, TModel>::value, int>::type
    inline static getLengthForMeasurement()
    {
        return 0;
    }

    template<typename TMeasurement>
    typename std::enable_if<SensorMeasurementConverter::is_support<TMeasurement, TModel>::value, int>::type
    inline static getLengthForMeasurement()
    {
        return TMeasurement::NumberMeasurement;
    }

    template<typename TMeasurement>
    typename std::enable_if<!SensorMeasurementConverter::is_support<TMeasurement, TModel>::value, void>::type
    AddMeasurement(TMeasurement& measurement)
    {
        std::cout << "this measurement is not supported" << std::endl;
        return;
    }
  //stack？  这是一个什么过程？是将所有类型的数据进行拼接
    template<typename TMeasurement>//测量量结构
    typename std::enable_if<SensorMeasurementConverter::is_support<TMeasurement, TModel>::value, void>::type
    AddMeasurement(TMeasurement& measurement)
    {
        if(!is_adding_measurement_) throw;
        if(stack_index_ + TMeasurement::NumberMeasurement > total_length_) throw;/////////////////////////////////////////////////////////////

        // stack h vector
        typename TMeasurement::MeasurementVector estimated_measurement;///////////////////////预测量测是这样的么？s_prediction
        SensorMeasurementConverter::getMeasurementPrediction<TMeasurement, TModel>(model_parameter_, state_prediction_, estimated_measurement);//具体量测函数里写的
        estimated_measurement_stack_.block<TMeasurement::NumberMeasurement, 1>(stack_index_, 0) = estimated_measurement;
        estimated_measurement_ = estimated_measurement;
        
        // stack H matrix
        Eigen::Matrix<double, TMeasurement::NumberMeasurement, TModel::NumberState> h_matrix;
        SensorMeasurementConverter::getMeasurementJacobian<TMeasurement, TModel>(model_parameter_, state_prediction_, h_matrix);
        h_matrix_stack_.block<TMeasurement::NumberMeasurement, TModel::NumberState>(stack_index_, 0) = h_matrix;

        // stack angle flag
        typename TMeasurement::MeasurementFlagVector flag;
        measurement.getAngleFlag(flag);
        angle_flag_stack_.block<TMeasurement::NumberMeasurement, 1>(stack_index_, 0) = flag;
        
        // stack measurement, measurement covariance & angle_flag
        double timestamp;
        typename TMeasurement::MeasurementVector measurement_vec;
        typename TMeasurement::MeasurementMatrix measurement_cov;
        measurement.getMeasurement(measurement_vec, measurement_cov, timestamp);
        measurement_stack_.block<TMeasurement::NumberMeasurement, 1>(stack_index_, 0) = measurement_vec;
        measurement_covariance_stack_.block<TMeasurement::NumberMeasurement, TMeasurement::NumberMeasurement>(stack_index_, stack_index_) = measurement_cov;
        
        stack_index_ += TMeasurement::NumberMeasurement;
    }

    void endAddMeasurement(typename TModel::FilterVector& state, typename TModel::FilterMatrix& state_covariance, double& time)
    {
        if(!is_adding_measurement_) throw;
        if(stack_index_ != total_length_) throw;

        is_adding_measurement_ = false;

        // begin kalman filter core
        // S = H*P(k+1|k)*H'  偏差的方差阵
        residual_covariance_ = (h_matrix_stack_ * state_prediction_covariance_ * h_matrix_stack_.transpose()) + measurement_covariance_stack_;
        // S_inv = (inv(S)+inv(S)')/2, to make symmetric
        residual_covariance_inverse_ = residual_covariance_.inverse();
        residual_covariance_inverse_ = (1/2.0) * (residual_covariance_inverse_ + residual_covariance_inverse_.transpose());
        // Check if S_inv is positive defined
        if(!isMatrixPositiveDefined(residual_covariance_inverse_))
        {
            std::cout << "residual_covariance_inverse_ is not semi-positive definitie, possible error" << std::endl;
        }
        // v = z-z_p
        measurement_residual_ = measurement_stack_ - estimated_measurement_stack_;
        //std::cout << measurement_residual_ << std::endl;

        // fix angle residual problem
        for(int i=0; i<total_length_; i++)
        {
            if(angle_flag_stack_(i) != 0)
            {
                measurement_residual_(i) = normalizeAngle(measurement_residual_(i));
            }
        }
        // W = P(k+1|k)*h'*S_inv
        filter_gain_ = state_prediction_covariance_ * (h_matrix_stack_.transpose() * residual_covariance_inverse_);
        // x = x_p+W*v
        state_ = state_prediction_ + filter_gain_ * measurement_residual_;
        // using origin covariance update method
        // state_ = state_prediction_ + filter_gain_ * measurement_residual_;
        // using joseph covariance update method for safe
        // C = (I-W*H)
        joseph_form_covariance_ = identity_p_ - filter_gain_ * h_matrix_stack_;
        // P(k+1|k+1) = C*P(k+1|k)*C' + W*R*W'
        state_covariance_ = joseph_form_covariance_ * state_prediction_covariance_ * joseph_form_covariance_.transpose() + filter_gain_ * measurement_covariance_stack_ * filter_gain_.transpose();
        // add a small eps to P(k+1|k+1)
        state_covariance_ = state_covariance_ + 0.0001 * identity_p_;///////////////////防止奇异///////////////////////////////////////////////////////////////
        // Check if P(k+1|k+1) is positive defined
        if(!isMatrixPositiveDefined(state_covariance_))
        {
            std::cout << "state_covariance_ is not semi-positive definitie, possible error" << std::endl;
        }
        state = state_;
        state_covariance = state_covariance_;
        W_=filter_gain_;

        time = timestamp_;
    }

    // supporting function
    template<typename InputType>
    inline bool isMatrixPositiveDefined(const Eigen::EigenBase<InputType> &matrix) const
    {
        Eigen::LLT<Eigen::MatrixXd> llt(matrix);
        return llt.info() == Eigen::Success;
    }

    // supporting function
    inline double normalizeAngle(const double raw_angle) const//象限问题
    {
        int n = 0;
        double angle = 0;
        n = raw_angle/(3.141592653 * 2);
        angle = raw_angle - (3.141592653 * 2) * n;
        if(angle > 3.141592653)
        {
            angle = angle - (3.141592653 * 2);
        }else if(angle <= -3.141592653)
        {
            angle = angle + (3.141592653 * 2);
        }
            
        return angle;
    }

    typename TModel::ModelParameter model_parameter_;
    Eigen::VectorXd estimated_measurement_;
    typename TModel::FilterVector state_pre_;
private:
    // vars for stack
    Eigen::VectorXd measurement_stack_;//动态矩阵，不断被扩充
    Eigen::VectorXd estimated_measurement_stack_;
    Eigen::VectorXd measurement_prediction_;////////////////////////其实没用？？？？？？？？？？
    Eigen::VectorXd measurement_residual_;
    Eigen::VectorXi angle_flag_stack_;
    Eigen::MatrixXd measurement_covariance_stack_;//R
    Eigen::MatrixXd h_matrix_stack_;
    Eigen::MatrixXd residual_covariance_;
    Eigen::MatrixXd residual_covariance_inverse_;
    Eigen::MatrixXd filter_gain_;
    bool is_adding_measurement_ = false;
    double timestamp_;
    int stack_index_;
    int total_length_;////每次更新时，用到的量测结果数量
    double delta_;

    //base
    typename TModel::FilterVector state_;
    typename TModel::FilterVector state_prediction_;
    typename TModel::FilterMatrix state_covariance_;
    typename TModel::FilterMatrix state_prediction_covariance_;
    typename TModel::FilterMatrix jacobian_;
    typename TModel::FilterMatrix process_noise_;
    typename TModel::FilterMatrix joseph_form_covariance_;
    typename TModel::FilterMatrix identity_p_;
};

#endif

