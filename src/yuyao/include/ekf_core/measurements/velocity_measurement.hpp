#ifndef MEASUREMENTS_VELOCITY_MEASUREMENT_HPP
#define MEASUREMENTS_VELOCITY_MEASUREMENT_HPP

// #include <string>
// #include <iostream>

#include "ekf_core/sensor_measurement_base.hpp"
#include "ekf_core/models/ctra.hpp"
#include "ekf_core/models/bicycle.hpp"


class VelocityMeasurement : public SensorMeasurementBase<1>
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
    struct is_support<VelocityMeasurement, CTRAModel> : std::true_type {};

    template<>
    struct is_support<VelocityMeasurement, BICYCLEModel> : std::true_type {};


    // template<>
    // void getMeasurementJacobian<VelocityMeasurement, AKERMANModel>(
    //     const typename AKERMANModel::ModelParameter& parameter,
    //     const typename AKERMANModel::FilterVector& state,
    //     Eigen::Matrix<double, VelocityMeasurement::NumberMeasurement, AKERMANModel::NumberState>& H
    // )
    // {
    //     H << 0, 0, 0, 1, 0, 0;
    // }

    // template<>
    // void getMeasurementPrediction<VelocityMeasurement, AKERMANModel>(
    //     const typename AKERMANModel::ModelParameter& parameter,
    //     const typename AKERMANModel::FilterVector& state,
    //     typename VelocityMeasurement::MeasurementVector& measurement
    // )
    // {
    //     measurement << state(3);
    // }


    template<>
    void getMeasurementJacobian<VelocityMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        Eigen::Matrix<double, VelocityMeasurement::NumberMeasurement, CTRAModel::NumberState>& H
    )
    {
        H << 0, 0, 0, 1, 0, 0;
    }

    template<>
    void getMeasurementPrediction<VelocityMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        typename VelocityMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(3);//////////////////////////提供输出当前估计速度的接口
    }


    template<>
    void getMeasurementJacobian<VelocityMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        Eigen::Matrix<double, VelocityMeasurement::NumberMeasurement, BICYCLEModel::NumberState>& H
    )
    {
        H << 0, 0, 0, -state(4) * sin(state(3)), cos(state(3)), 0;
    }////////////////////////////////////////////////////得到的速度是车轴速度，而状态v是实际速度

    template<>
    void getMeasurementPrediction<VelocityMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        typename VelocityMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(4) * cos(state(3));
    }
}

#endif
