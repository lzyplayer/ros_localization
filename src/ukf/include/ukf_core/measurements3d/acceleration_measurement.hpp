#ifndef MEASUREMENTS_ACCELERATION_MEASUREMENT_HPP
#define MEASUREMENTS_ACCELERATION_MEASUREMENT_HPP

#include "ukf_core/sensor_measurement_base.hpp"
#include "ukf_core/models/smodel_ukf3d.hpp"

//#include "ukf_core/models/bicyclepitch.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
class AccelerationMeasurement : public SensorMeasurementBase<3>
{
public:
    void getAngleFlag(
        MeasurementFlagVector& angle_flag
    ) const override final
    {
        angle_flag << 0, 0, 0;
    }
    static const int NumberMeasurement = 3;
};

namespace SensorMeasurementConverter
{

    template<>
    struct is_support<AccelerationMeasurement, SModel> : std::true_type {};



    template<>
    void getMeasurementPrediction<AccelerationMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const Eigen::MatrixXd& Xsig_pred,
        Eigen::MatrixXd& Zsig,
        typename AccelerationMeasurement::MeasurementVector& measurement_pred
    )//sigma points
    {
        int n_sig = Xsig_pred.cols();
        Zsig = Xsig_pred.block(10, 0, 3, n_sig);
        measurement_pred.setZero();
    }

    template<>
    void getMeasurementPub<AccelerationMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const typename SModel::FilterVector& state,
        typename AccelerationMeasurement::MeasurementVector& measurement
    )
    {
        Vector3d A_ = state.middleRows(10, 3);
        // Vector3d A_bias = state.middleRows(16, 3);
        // Eigen::Quaternion<double> q(state[6], state[7], state[8], state[9]);
        // q.normalize();
        // Vector3d g;
        // g << 0.0f, 0.0f, -9.80665f;
    
        measurement = A_; //测量的是实际加速度在车轴方向的一个分量，在2D情况下
    }
}

#endif
