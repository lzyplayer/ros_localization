#ifndef UNSCENTED_KALMAN_FILTER_HPP
#define UNSCENTED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Core>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

#define PI           3.14159265358979323846  /* pi */
template<typename TModel>//模型类型，参数是 类 名称
class UnscentedKalmanFilter
{
public:
    double lambda = -8;
    typedef TModel Model;
    UnscentedKalmanFilter(const typename TModel::ModelParameter& para)
    :
    model_parameter_(para)
    {
        N = TModel::NumberState;//增广后
        S = 2 * N + 1;
        weights.resize(S, 1);
        Xsig.resize(N, S);
        Xsig_prediction.resize(N, S);
        weights(0) = lambda / (N + lambda);
        for (int i = 1; i < S; i++) 
        {
            weights(i) = 1 / (2 * (N + lambda));
        }
    }

    void init(double timestamp)//被调用
    {
        identity_p_.setIdentity();//单位阵
        TModel::initState(model_parameter_, x_, P_);
        timestamp_ = timestamp;
    }

    void beginAddMeasurement(double timestamp, int total_length, bool flag)//当前测量的变量数
    {
        if(is_adding_measurement_) throw;

        // resize the matrixes to fit the input匹配大小
        measurement_stack_.resize(total_length);//eigen库函数
        measurement_prediction_stack_.resize(total_length);
        h_matrix_stack_.resize(total_length, TModel::NumberState);
        S_.resize(total_length, total_length);
        measurement_covariance_stack_.resize(total_length, total_length);
        measurement_covariance_stack_.setZero();
        //measurement_prediction_.resize(total_length);
        //measurement_residual_.resize(total_length);
        //K_.resize(TModel::NumberState, total_length);
        angle_flag_stack_.resize(total_length);
        
        stack_index_ = 0;
        stack_index1_ = 0;
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
        // begin cycle 得到x(k+1|k) P(k+1|k)
        predict(delta_);//静态变量函数，所以可以直接通过类来引用
    }
    
    void predict(const double& delta)
    {
        Xsig_prediction.setZero();
        //generate sigma_point
        ensurePositiveFinite(P_);
        GenerateSigmaPoints(x_, P_, Xsig);
        std::cout << "P:" << P_ << std::endl;


        //predict sigma_points
        for (int i = 0;  i < S; i++)
        {
            typename TModel::FilterVector in = Xsig.col(i);
            typename TModel::FilterVector out = Xsig_prediction.col(i);
            TModel::updatePrediction(model_parameter_, in, delta, out);
            Xsig_prediction.col(i) = out;
            // std::cout << "in:" << in << std::endl;
        }
    
        //compute x_pre
        x_pre.resize(x_.size());
        P_pred.resize(P_.rows(), P_.cols());
        x_pre.setZero();
        P_pred.setZero();
        for (int i = 0; i < S; i++) 
        {
            x_pre += weights(i) * Xsig_prediction.col(i); 
        }
        for (int i = 0; i < S; i++) 
        {
            VectorXd diff = Xsig_prediction.col(i) - x_;
            if(i==0)  P_pred += (3+1/(N+lambda)) * diff * diff.transpose();
            else  P_pred += weights(i) * diff * diff.transpose();
        }                    
        P_pred += model_parameter_.process_noise;
        // std::cout << "xpre:" << x_pre << std::endl;
        ensurePositiveFinite(P_pred);
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
    template<typename TMeasurement>//测量量结构
    typename std::enable_if<SensorMeasurementConverter::is_support<TMeasurement, TModel>::value, void>::type
    AddMeasurement(TMeasurement& measurement)
    {
        if(!is_adding_measurement_) throw;
        if(stack_index_ + 1 > total_length_) throw;/////////////////////////////////////////////////////////////

        // stack angle flag
        typename TMeasurement::MeasurementFlagVector flag;
        measurement.getAngleFlag(flag);
        angle_flag_stack_.block(stack_index_, 0, TMeasurement::NumberMeasurement, 1) = flag;
        
        // stack measurement, measurement covariance & angle_flag
        double timestamp;
        typename TMeasurement::MeasurementVector measurement_vec;
        typename TMeasurement::MeasurementMatrix measurement_cov;
        measurement.getMeasurement(measurement_vec, measurement_cov, timestamp);
        measurement_stack_.block(stack_index_, 0, TMeasurement::NumberMeasurement, 1) = measurement_vec;
        measurement_covariance_stack_.block<TMeasurement::NumberMeasurement, TMeasurement::NumberMeasurement>(stack_index_, stack_index_) = measurement_cov;
        
        stack_index_ += TMeasurement::NumberMeasurement; 
    }

    template<typename TMeasurement>//测量量结构
    typename std::enable_if<SensorMeasurementConverter::is_support<TMeasurement, TModel>::value, void>::type
    AddMeasurement2(TMeasurement& measurement)
    {
        if(stack_index1_ + 1 > total_length_) throw;/////////////////////////////////////////////////////////////
        // stack h vector
        typename TMeasurement::MeasurementVector measurement_prediction;
        MatrixXd Zsig1;
        int S_ = M;
        Zsig1.resize(TMeasurement::NumberMeasurement, S_);
        SensorMeasurementConverter::getMeasurementPrediction<TMeasurement, TModel>(model_parameter_, ext_sig, Zsig1, measurement_prediction);//具体量测函数里写的
        Zsig_stack_.block(stack_index1_, 0, TMeasurement::NumberMeasurement, S_) = Zsig1;
        stack_index1_ += TMeasurement::NumberMeasurement;
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void midAddMeasurement()
    {
        if(!is_adding_measurement_) throw;
        if(stack_index_ != total_length_) throw;

        is_adding_measurement_ = false;

        int K = total_length_;
        ext_mean_pred = VectorXd::Zero(N + K, 1);
        ext_cov_pred = MatrixXd::Zero(N + K, N + K);
        ext_mean_pred.topLeftCorner(N, 1) = x_pre;//已经经过预测
        ext_cov_pred.topLeftCorner(N, N) = P_pred;
        ext_cov_pred.bottomRightCorner(K, K) = measurement_covariance_stack_;
        ensurePositiveFinite(ext_cov_pred);
        M = 2 * (N + K) +1;
        ext_sig.resize(N + K, M);
        GenerateSigmaPoints(ext_mean_pred, ext_cov_pred, ext_sig);
        //std::cout << "ext_cov_pred:" << ext_cov_pred << std::endl;
        Zsig.resize(K, M);
        for(int i = 0; i < K; i++)
        {
            Zsig.row(i) = ext_sig.row(i + N);
        }
        ext_weights.resize(M, 1);
        double lambda_ = -8-K;
        ext_weights(0) = lambda_ / (N + K + lambda_);
        for (int i = 1; i < M; i++) 
        {
            ext_weights(i) = 1 / (2 * (N + K + lambda_));
        }
        ext_sig_mean = VectorXd::Zero(N + K, 1);
        for (int i = 1; i < M; i++) 
        {
            ext_sig_mean += ext_weights[i] * ext_sig.col(i);
        }
        // std::cout << "midwancheng" << std::endl;
        Zsig_stack_.resize(total_length_, M);
    }

    void endAddMeasurement(typename TModel::FilterVector& state, typename TModel::FilterMatrix& state_covariance, double& time)
    {
        Zsig += Zsig_stack_;
        int K = total_length_;
        VectorXd expected_measurement_mean = VectorXd::Zero(K);
        for (int i = 0; i < M; i++)
        {
            expected_measurement_mean += ext_weights[i] * Zsig.col(i);
        }
        MatrixXd expected_measurement_cov = MatrixXd::Zero(K, K);
        for (int i = 0; i < M; i++) 
        {
            VectorXd diff = Zsig.col(i) - expected_measurement_mean;
            // expected_measurement_cov += ext_weights[i] * diff * diff.transpose();
            if(i==0)  expected_measurement_cov += (3+1/(N+lambda+K)) * diff * diff.transpose();
            else  expected_measurement_cov += ext_weights(i) * diff * diff.transpose();
        }
            // calculated transformed covariance
        MatrixXd sigma = MatrixXd::Zero(N + K, K);
        for (int i = 0; i < ext_sig.rows(); i++) 
        {
            auto diffA = (ext_sig.col(i) - ext_mean_pred);
            auto diffB = (Zsig.col(i) - expected_measurement_mean);
            // sigma += ext_weights[i] * (diffA * diffB.transpose());
            if(i==0)  sigma += (3+1/(N+lambda+K)) * (diffA * diffB.transpose());
            else  sigma += ext_weights(i) * (diffA * diffB.transpose());
        }
        kalman_gain = sigma * expected_measurement_cov.inverse();

        VectorXd measurement_residual_ = measurement_stack_ - expected_measurement_mean;
        for(int i=0; i<total_length_; i++)
        {
            if(angle_flag_stack_(i) != 0)
            {
                measurement_residual_(i) = normalizeAngle(measurement_residual_(i));
            }
        }
        VectorXd ext_mean = ext_mean_pred + kalman_gain * measurement_residual_;
        // cout << "res" << measurement_stack_ <<endl<< expected_measurement_mean << endl;
        MatrixXd ext_cov = ext_cov_pred - kalman_gain * expected_measurement_cov * kalman_gain.transpose();

        x_ = ext_mean.topLeftCorner(N, 1);
        P_ = ext_cov.topLeftCorner(N, N);
        ensurePositiveFinite(P_);
        // std::cout << "P:" << P_ << std::endl;
        // std::cout << "X:" << x_ << std::endl;
        state = x_;
        state_covariance = P_;
        time = timestamp_;
    }


public:
    typename TModel::ModelParameter model_parameter_;
    MatrixXd Xsig;
    MatrixXd ext_sig;
    VectorXd weights;
    VectorXd ext_weights;
    VectorXd x_pre;
    VectorXd ext_mean_pred;
    VectorXd ext_sig_mean;
    MatrixXd ext_cov_pred;
    MatrixXd kalman_gain;
    MatrixXd P_pred;

    typename TModel::FilterMatrix Q_;
    Eigen::MatrixXd W_;

    int N;//state_dim
    int S;
    int M;
private:

    // vars for stack
    Eigen::VectorXd measurement_stack_;//动态矩阵，不断被扩充
    Eigen::VectorXd measurement_prediction_stack_;
    Eigen::VectorXd measurement_residual_;
    Eigen::VectorXi angle_flag_stack_;
    Eigen::MatrixXd measurement_covariance_stack_;//R
    Eigen::MatrixXd h_matrix_stack_;
    Eigen::MatrixXd S_;
    Eigen::MatrixXd S_inverse_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd Zsig;
    Eigen::MatrixXd Zsig_stack_;
    Eigen::MatrixXd Xsig_prediction;
    bool is_adding_measurement_ = false;
    double timestamp_;
    int stack_index_;
    int stack_index1_;
    int total_length_;////每次更新时，用到的量测结果数量
    double delta_;
    //base
    typename TModel::FilterVector x_;
    
    typename TModel::FilterVector x_prediction_;
    Eigen::MatrixXd P_;
    typename TModel::FilterMatrix P_prediction_;

    typename TModel::FilterMatrix identity_p_;
    
    void GenerateSigmaPoints(const VectorXd& x, const MatrixXd& P, MatrixXd& Xsig_in)
    {
        int n = x.size();//增广之后、增广之前
        assert(P.rows() == n && P.cols() == n);
        //MatrixXd A = P.llt().matrixL();
        Eigen::LLT<MatrixXd> llt;
        llt.compute((n + lambda) * P);
        MatrixXd l = llt.matrixL();
        Xsig_in.col(0) = x;
        for (int i = 0; i < n; i++)
        {
            Xsig_in.col(i + 1) = x + l.col(i);
            Xsig_in.col(i + n + 1) = x -  l.col(i);
        }
    }

    void ensurePositiveFinite(MatrixXd& cov) 
    {
        const double eps = 1e-9;

        Eigen::EigenSolver<MatrixXd> solver(cov);
        MatrixXd D = solver.pseudoEigenvalueMatrix();
        MatrixXd V = solver.pseudoEigenvectors();
        for (int i = 0; i < D.rows(); i++) 
        {
            if (D(i, i) < eps) 
            {
                D(i, i) = eps;
            }
        }

        cov = V * D * V.inverse();
        return;
    }

        // supporting function
    template<typename InputType>
    inline bool isMatrixPositiveDefined(const Eigen::EigenBase<InputType> &matrix) const
    {
        Eigen::LLT<Eigen::MatrixXd> llt(matrix);
        return llt.info() == Eigen::Success;
    }
public:
    // supporting function
    static double normalizeAngle(const double raw_angle)//象限问题
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
};
#endif

