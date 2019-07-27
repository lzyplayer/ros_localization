#ifndef MEASUREMENTS_STEERINGWHEELANGLE_MEASUREMENT_HPP
#define MEASUREMENTS_STEERINGWHEELANGLE_MEASUREMENT_HPP

#include "ekf_core/sensor_measurement_base.hpp"
#include "ekf_core/models/ctra.hpp"
#include "ekf_core/models/bicycle.hpp"

class SteeringwheelangleMeasurement : public SensorMeasurementBase<1>
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
    // struct is_support<SteeringwheelangleMeasurement, AKERMANModel> : std::true_type {};

    template<>
    struct is_support<SteeringwheelangleMeasurement, CTRAModel> : std::true_type {};

    template<>
    struct is_support<SteeringwheelangleMeasurement, BICYCLEModel> : std::true_type {};

    // template<>
    // void getMeasurementJacobian<SteeringwheelangleMeasurement, AKERMANModel>(
    //     const typename AKERMANModel::ModelParameter& parameter,
    //     const typename AKERMANModel::FilterVector& state,
    //     Eigen::Matrix<double, SteeringwheelangleMeasurement::NumberMeasurement, AKERMANModel::NumberState>& H
    // )
    // {
    //     double L=parameter.L;
    //     double w=state(4);
    //     double fm=state(3)*state(3)+w*w*L*L;
    //     H << 0, 0, 0, -w*L/(fm), state(3)*L/fm, 0;
    // }

    // template<>
    // void getMeasurementPrediction<SteeringwheelangleMeasurement, AKERMANModel>(
    //     const typename AKERMANModel::ModelParameter& parameter,
    //     const typename AKERMANModel::FilterVector& state,
    //     typename SteeringwheelangleMeasurement::MeasurementVector& measurement
    // )
    // {
    //     measurement << atan2(parameter.L * state(4), state(3));
    // } 


    template<>
    void getMeasurementJacobian<SteeringwheelangleMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        Eigen::Matrix<double, SteeringwheelangleMeasurement::NumberMeasurement, CTRAModel::NumberState>& H
    )//用状态表示量测，不一定可以写成矩阵相乘形式，再求偏导得到H矩阵
    {
        H << 0, 0, 0, -(parameter.l * state(4))/(parameter.l * parameter.l * state(4) * state(4) + state(3) * state(3)), (parameter.l * state(3))/(parameter.l * parameter.l * state(4) * state(4) + state(3) * state(3)), 0;
        
    }

    template<>
    void getMeasurementPrediction<SteeringwheelangleMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        typename SteeringwheelangleMeasurement::MeasurementVector& measurement
    )
    {
        measurement << atan2((parameter.l) * state(4), state(3));
        
    }


    template<>
    void getMeasurementJacobian<SteeringwheelangleMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        Eigen::Matrix<double, SteeringwheelangleMeasurement::NumberMeasurement, BICYCLEModel::NumberState>& H
    )//用状态表示量测，不一定可以写成矩阵相乘形式，再求偏导得到H矩阵
    {
        H << 0, 0, 0, (parameter.lr * (parameter.lf + parameter.lr))/(parameter.lr * parameter.lr * cos(state(3)) * cos(state(3)) + (parameter.lf + parameter.lr) * (parameter.lf + parameter.lr) * sin(state(3)) * sin(state(3))), 0, 0;
        
    }

    template<>
    void getMeasurementPrediction<SteeringwheelangleMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        typename SteeringwheelangleMeasurement::MeasurementVector& measurement
    )
    {
        measurement << atan2((parameter.lf + parameter.lr) * sin(state(3)), parameter.lr * cos(state(3)));
        
    }
}

#endif