#ifndef MEASUREMENTS_VELOCITY_MEASUREMENT_HPP
#define MEASUREMENTS_VELOCITY_MEASUREMENT_HPP

#include "ukf_core/sensor_measurement_base.hpp"
#include "ukf_core/models/smodel_ukf2d.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
class VelocityMeasurement : public SensorMeasurementBase<2>
{
public:
    void getAngleFlag(
        MeasurementFlagVector& angle_flag
    ) const override final
    {
        angle_flag << 0, 0;
    }
    static const int NumberMeasurement = 2;
};

namespace SensorMeasurementConverter
{

    template<>
    struct is_support<VelocityMeasurement, SModel> : std::true_type {};

    template<>
    void getMeasurementPrediction<VelocityMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const Eigen::MatrixXd& Xsig_pred,
        Eigen::MatrixXd& Zsig,
        typename VelocityMeasurement::MeasurementVector& measurement_pred
    )
    {
        int n_sig = Xsig_pred.cols();
        Zsig = Xsig_pred.block(2, 0, 2, n_sig);
        measurement_pred.setZero();
    }

    template<>
    void getMeasurementPub<VelocityMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const typename SModel::FilterVector& state,
        typename VelocityMeasurement::MeasurementVector& measurement
    )
    {
        Vector2d V_ = state.middleRows(2, 2);

        measurement = V_ ; //测量的是实际加速度在车轴方向的一个分量，在2D情况下
    }
}

#endif
