#ifndef MEASUREMENTS_ANGULARVELOCITY_MEASUREMENT_HPP
#define MEASUREMENTS_ANGULARVELOCITY_MEASUREMENT_HPP

#include "ekf_core/sensor_measurement_base.hpp"
#include "ekf_core/models/ctra.hpp"
#include "ekf_core/models/bicycle.hpp"

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
    // template<>
    // struct is_support<AngularvelocityMeasurement, AKERMANModel> : std::true_type {};

    template<>
    struct is_support<AngularvelocityMeasurement, CTRAModel> : std::true_type {};

    template<>
    struct is_support<AngularvelocityMeasurement, BICYCLEModel> : std::true_type {};

    
    // template<>
    // void getMeasurementJacobian<AngularvelocityMeasurement, AKERMANModel>(
    //     const typename AKERMANModel::ModelParameter& parameter,
    //     const typename AKERMANModel::FilterVector& state,
    //     Eigen::Matrix<double, AngularvelocityMeasurement::NumberMeasurement, AKERMANModel::NumberState>& H
    // )
    // {
    //     //H << 0, 0, 0, 1/state(4), -state(3)/(state(4)*state(4)), 0;
    //     H << 0, 0, 0, 0, 1, 0;
    // }

    // template<>
    // void getMeasurementPrediction<AngularvelocityMeasurement, AKERMANModel>(
    //     const typename AKERMANModel::ModelParameter& parameter,
    //     const typename AKERMANModel::FilterVector& state,
    //     typename AngularvelocityMeasurement::MeasurementVector& measurement
    // )
    // {
    //     measurement << state(4);
    // }

    template<>
    void getMeasurementJacobian<AngularvelocityMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        Eigen::Matrix<double, AngularvelocityMeasurement::NumberMeasurement, CTRAModel::NumberState>& H
    )
    {
        H << 0, 0, 0, 0, 1, 0;
    }

    template<>
    void getMeasurementPrediction<AngularvelocityMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        typename AngularvelocityMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(4);
    }


    template<>
    void getMeasurementJacobian<AngularvelocityMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        Eigen::Matrix<double, AngularvelocityMeasurement::NumberMeasurement, BICYCLEModel::NumberState>& H
    )
    {
        H << 0, 0, 0, (state(4) * cos(state(3)))/parameter.lr, sin(state(3))/parameter.lr, 0;
    }

    template<>
    void getMeasurementPrediction<AngularvelocityMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        typename AngularvelocityMeasurement::MeasurementVector& measurement
    )
    {
        measurement << (state(4) * sin(state(3)))/parameter.lr;
    }

}

#endif
