#ifndef MEASUREMENTS_ANGULARVELOCITY_MEASUREMENT_HPP
#define MEASUREMENTS_ANGULARVELOCITY_MEASUREMENT_HPP

#include "ukf_core/sensor_measurement_base.hpp"

#include "ukf_core/models/smodel_ukf2d.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
class AngularvelocityMeasurement : public SensorMeasurementBase<1>
{
public:
    void getAngleFlag(
        MeasurementFlagVector& angle_flag
    ) const override final
    {
        angle_flag << 0;
    }
};

namespace SensorMeasurementConverter
{

    template<>
    struct is_support<AngularvelocityMeasurement, SModel> : std::true_type {};


    template<>
    void getMeasurementPrediction<AngularvelocityMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const Eigen::MatrixXd& Xsig_pred,
        Eigen::MatrixXd& Zsig,
        typename AngularvelocityMeasurement::MeasurementVector& measurement_pred
    )
    {
        int n_sig = Xsig_pred.cols();
        Zsig = Xsig_pred.middleRows(7, 1);
        measurement_pred.setZero();
    }


    template<>
    void getMeasurementPub<AngularvelocityMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const typename SModel::FilterVector& state,
        typename AngularvelocityMeasurement::MeasurementVector& measurement
    )
    {
        measurement = state.middleRows(7, 1);
    }
}

#endif
