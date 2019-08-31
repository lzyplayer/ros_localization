#ifndef KALMAN_FILTER_MODEL_BASE_HPP
#define KALMAN_FILTER_MODEL_BASE_HPP
//防止头文件多重调用，一般都要将头文件写在格式里
struct KalmanFilterModelParameterBase
{
    
};

template<int NState, typename TModelParameter = KalmanFilterModelParameterBase>//类模板
class KalmanFilterModelBase
{
public:
    typedef TModelParameter ModelParameter;
    const static int NumberState = NState;

    typedef Eigen::Matrix<double, NState, 1> FilterVector;//变量类型名称替换
    typedef Eigen::Matrix<double, NState, NState> FilterMatrix;

    static void initState(
        const TModelParameter& model_parameter,
        FilterVector& state,
        FilterMatrix& state_covariance
    );//只有参数的空函数

    static void updatePrediction(
        const TModelParameter& model_parameter,
        const FilterVector& state,
        const double delta,
        FilterVector& state_prediction,
        FilterMatrix& jacobian,
        FilterMatrix& process_noise
    );
    static double NormalizeAngle(
        const double raw_angle
    )
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
    };
};

#endif
