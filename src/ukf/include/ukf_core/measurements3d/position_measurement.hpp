#ifndef MEASUREMENTS_POSITION_MEASUREMENT_HPP
#define MEASUREMENTS_POSITION_MEASUREMENT_HPP

#include "ukf_core/sensor_measurement_base.hpp"

#include "ukf_core/models/smodel_ukf3d.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
class PositionMeasurement : public SensorMeasurementBase<3>
{
public:
    void getAngleFlag(
        MeasurementFlagVector& angle_flag
    ) const override final
    {
        angle_flag << 0, 0, 0;
    }
};

namespace SensorMeasurementConverter
{


    template<>
    struct is_support<PositionMeasurement, SModel> : std::true_type {};


    template<>
    void getMeasurementPrediction<PositionMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const Eigen::MatrixXd& Xsig_pred,
        Eigen::MatrixXd& Zsig,
        typename PositionMeasurement::MeasurementVector& measurement_pred
    )
    {
        int n_sig = Xsig_pred.cols();
        Zsig = Xsig_pred.block(0, 0, 3, n_sig);
        measurement_pred.setZero();
        // for (int i = 0; i < n_sig; i++) 
        // {
        //     measurement_pred = measurement_pred + parameter.weights_m_(i) * Z_sig.col(i);
        // }
        // for (int i = 0; i < n_sig; i++) 
        // {
        //     Zsig.col(i) = Z_sig.col(i) - measurement_pred;
        // }
    }

    template<>
    void getMeasurementPub<PositionMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const typename SModel::FilterVector& state,
        typename PositionMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(0), state(1), state(2);
    }
}

#endif
