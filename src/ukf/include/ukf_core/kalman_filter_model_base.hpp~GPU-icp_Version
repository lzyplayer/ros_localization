#ifndef KALMAN_FILTER_MODEL_BASE_HPP
#define KALMAN_FILTER_MODEL_BASE_HPP
//防止头文件多重调用，一般都要将头文件写在格式里
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct KalmanFilterModelParameterBase
{
    
};

template<int NState, typename TModelParameter = KalmanFilterModelParameterBase>//类模板
class KalmanFilterModelBase
{
public:
    typedef TModelParameter ModelParameter;
    static const int NumberState = NState;


    typedef Eigen::Matrix<double, NState, 1> FilterVector;//变量类型名称替换
    typedef Eigen::Matrix<double, NState, NState> FilterMatrix;
    typedef Eigen::Matrix<double, NState, 2*NState+5> FilterMatrixSig;



    static void initState(
        const TModelParameter& model_parameter,
        VectorXd& state,
        MatrixXd& state_covariance
    );//只有参数的空函数

    static void updatePrediction(
        const ModelParameter& parameter,
        const FilterVector xsig,
        const double delta,
        FilterVector& xsig_prediction
    );
};

#endif
