#ifndef MEASUREMENTS_POSITION_MEASUREMENT_HPP
#define MEASUREMENTS_POSITION_MEASUREMENT_HPP

#include "ekf_core/sensor_measurement_base.hpp"
#include "ekf_core/models/ctra.hpp"
#include "ekf_core/models/akerman.hpp"
#include "ekf_core/models/bicycle.hpp"

class PositionMeasurement : public SensorMeasurementBase<2>
{
public:
    void getAngleFlag(
        MeasurementFlagVector& angle_flag
    ) const override final
    {
        angle_flag << 0, 0;
    }
};

namespace SensorMeasurementConverter
{
    template<>
    struct is_support<PositionMeasurement, AKERMANModel> : std::true_type {};

    template<>
    struct is_support<PositionMeasurement, CTRAModel> : std::true_type {};

    template<>
    struct is_support<PositionMeasurement, BICYCLEModel> : std::true_type {};

    template<>
    void getMeasurementJacobian<PositionMeasurement, AKERMANModel>(
        const typename AKERMANModel::ModelParameter& parameter,
        const typename AKERMANModel::FilterVector& state,
        Eigen::Matrix<double, PositionMeasurement::NumberMeasurement, AKERMANModel::NumberState>& H
    )
    {
        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0;
    }

    template<>
    void getMeasurementPrediction<PositionMeasurement, AKERMANModel>(
        const typename AKERMANModel::ModelParameter& parameter,
        const typename AKERMANModel::FilterVector& state,
        typename PositionMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(0), state(1);//归到后轮中心
    }

    template<>
    void getMeasurementJacobian<PositionMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        Eigen::Matrix<double, PositionMeasurement::NumberMeasurement, CTRAModel::NumberState>& H
    )
    {
        H << 1, 0, -1.02 * sin(state(2)), 0, 0, 0,
             0, 1, 1.02 * cos(state(2)), 0, 0, 0;
    //    H << 1, 0, 0, 0, 0, 0,
      //       0, 1, 0, 0, 0, 0;
    }

    template<>
    void getMeasurementPrediction<PositionMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        typename PositionMeasurement::MeasurementVector& measurement
    )
    {
        //measurement << state(0), state(1);
        measurement << state(0) + 1.02*cos(state(2)), state(1) +1.02*sin(state(2));
    }


    template<>
    void getMeasurementJacobian<PositionMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        Eigen::Matrix<double, PositionMeasurement::NumberMeasurement, BICYCLEModel::NumberState>& H
    )/////////////////////////////////////////////////////////////////////////////////////
    {
        // H << 1, 0, parameter.lr * sin(state(2)), 0, 0, 0,
        //      0, 1, -parameter.lr * cos(state(2)), 0, 0, 0;
        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0;
    }

    template<>
    void getMeasurementPrediction<PositionMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        typename PositionMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(0), state(1);
        // measurement << state(0) - parameter.lr * cos(state(2)), state(1) - parameter.lr * sin(state(2));
    }
}

#endif
