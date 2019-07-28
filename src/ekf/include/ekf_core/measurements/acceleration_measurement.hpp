#ifndef MEASUREMENTS_ACCELERATION_MEASUREMENT_HPP
#define MEASUREMENTS_ACCELERATION_MEASUREMENT_HPP

#include "ekf_core/sensor_measurement_base.hpp"
#include "ekf_core/models/ctra.hpp"
#include "ekf_core/models/akerman.hpp"
#include "ekf_core/models/bicycle.hpp"

class AccelerationMeasurement : public SensorMeasurementBase<1>
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
    struct is_support<AccelerationMeasurement, AKERMANModel> : std::true_type {};

    template<>
    struct is_support<AccelerationMeasurement, CTRAModel> : std::true_type {};

    template<>
    struct is_support<AccelerationMeasurement, BICYCLEModel> : std::true_type {};

    
    template<>
    void getMeasurementJacobian<AccelerationMeasurement, AKERMANModel>(
        const typename AKERMANModel::ModelParameter& parameter,
        const typename AKERMANModel::FilterVector& state,
        Eigen::Matrix<double, AccelerationMeasurement::NumberMeasurement, AKERMANModel::NumberState>& H
    )
    {
        //与后轴中心的加速度不同，目前先近似相同
        H << 0, 0, 0, 0, 0, 1;
    }

    template<>
    void getMeasurementPrediction<AccelerationMeasurement, AKERMANModel>(
        const typename AKERMANModel::ModelParameter& parameter,
        const typename AKERMANModel::FilterVector& state,
        typename AccelerationMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(5);
    }


    template<>
    void getMeasurementJacobian<AccelerationMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        Eigen::Matrix<double, AccelerationMeasurement::NumberMeasurement, CTRAModel::NumberState>& H
    )
    {
        H << 0, 0, 0, 0, 0, 1;
    }

    template<>
    void getMeasurementPrediction<AccelerationMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        typename AccelerationMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(5);
    }


    template<>
    void getMeasurementJacobian<AccelerationMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        Eigen::Matrix<double, AccelerationMeasurement::NumberMeasurement, BICYCLEModel::NumberState>& H
    )
    {
        const double& T_ = state(2);
        const double& B_ = state(3);
        const double& V_ = state(4);
        const double& A_ = state(5);

        double LR_ = parameter.lr;

        H << 0, 0, 0, -A_ * sin(B_), 0, cos(B_);
    }

    template<>
    void getMeasurementPrediction<AccelerationMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        typename AccelerationMeasurement::MeasurementVector& measurement
    )
    {
        const double& T_ = state(2);
        const double& B_ = state(3);
        const double& V_ = state(4);
        const double& A_ = state(5);
        
        double LR_ = parameter.lr;

        measurement << A_ * cos(B_);//测量的是实际加速度在车轴方向的一个分量，在2D情况下
    }

}

#endif
