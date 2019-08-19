#ifndef MEASUREMENTS_ACCELERATION_MEASUREMENT_HPP
#define MEASUREMENTS_ACCELERATION_MEASUREMENT_HPP

#include "ukf_core/sensor_measurement_base.hpp"
#include "ukf_core/models/smodel_ukf2d.hpp"

//#include "ukf_core/models/bicyclepitch.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
class AccelerationMeasurement : public SensorMeasurementBase<2>
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
    struct is_support<AccelerationMeasurement, SModel> : std::true_type {};



    template<>
    void getMeasurementPrediction<AccelerationMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const Eigen::MatrixXd& Xsig_pred,
        Eigen::MatrixXd& Zsig,
        typename AccelerationMeasurement::MeasurementVector& measurement_pred
    )
    {
        int n_sig = Xsig_pred.cols();
        Zsig = Xsig_pred.block(5, 0, 2, n_sig);
        measurement_pred.setZero();
    }

    template<>
    void getMeasurementPub<AccelerationMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const typename SModel::FilterVector& state,
        typename AccelerationMeasurement::MeasurementVector& measurement
    )
    {
        Vector2d A_ = state.middleRows(5, 2);

        measurement = A_ ; //测量的是实际加速度在车轴方向的一个分量，在2D情况下
    }
}

#endif
