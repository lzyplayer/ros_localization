#ifndef MEASUREMENTS_HEADING_MEASUREMENT_HPP
#define MEASUREMENTS_HEADING_MEASUREMENT_HPP

#include "ekf_core/sensor_measurement_base.hpp"
#include "ekf_core/models/ctra.hpp"
#include "ekf_core/models/akerman.hpp"
#include "ekf_core/models/bicycle.hpp"
#include "ekf_core/models/bicyclelr.hpp"

class HeadingMeasurement : public SensorMeasurementBase<1>
{
public:
    void getAngleFlag(
        MeasurementFlagVector& angle_flag
    ) const override final
    {
        angle_flag << 1;
    }
};

namespace SensorMeasurementConverter
{
    template<>
    struct is_support<HeadingMeasurement, AKERMANModel> : std::true_type {};

    template<>
    struct is_support<HeadingMeasurement, CTRAModel> : std::true_type {};

    template<>
    struct is_support<HeadingMeasurement, BICYCLEModel> : std::true_type {};

    template<>
    struct is_support<HeadingMeasurement, BICYCLELRModel> : std::true_type {};

    template<>
    void getMeasurementJacobian<HeadingMeasurement, AKERMANModel>(
        const typename AKERMANModel::ModelParameter& parameter,
        const typename AKERMANModel::FilterVector& state,
        Eigen::Matrix<double, HeadingMeasurement::NumberMeasurement, AKERMANModel::NumberState>& H
    )
    {
        H << 0, 0, 1, 0, 0, 0;
    }

    template<>
    void getMeasurementPrediction<HeadingMeasurement, AKERMANModel>(
        const typename AKERMANModel::ModelParameter& parameter,
        const typename AKERMANModel::FilterVector& state,
        typename HeadingMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(2);
    }    


    template<>
    void getMeasurementJacobian<HeadingMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        Eigen::Matrix<double, HeadingMeasurement::NumberMeasurement, CTRAModel::NumberState>& H
    )
    {
        H << 0, 0, 1, 0, 0, 0;
    }

    template<>
    void getMeasurementPrediction<HeadingMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        typename HeadingMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(2);
    }


    template<>
    void getMeasurementJacobian<HeadingMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        Eigen::Matrix<double, HeadingMeasurement::NumberMeasurement, BICYCLEModel::NumberState>& H
    )
    {
        H << 0, 0, 1, 0, 0, 0;
    }

    template<>
    void getMeasurementPrediction<HeadingMeasurement, BICYCLEModel>(
        const typename BICYCLEModel::ModelParameter& parameter,
        const typename BICYCLEModel::FilterVector& state,
        typename HeadingMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(2);//车轴朝向
    }

    template<>
    void getMeasurementJacobian<HeadingMeasurement, BICYCLELRModel>(
        const typename BICYCLELRModel::ModelParameter& parameter,
        const typename BICYCLELRModel::FilterVector& state,
        Eigen::Matrix<double, HeadingMeasurement::NumberMeasurement, BICYCLELRModel::NumberState>& H
    )
    {
        H << 0, 0, 0, 1, 0, 0, 0;
    }

    template<>
    void getMeasurementPrediction<HeadingMeasurement, BICYCLELRModel>(
        const typename BICYCLELRModel::ModelParameter& parameter,
        const typename BICYCLELRModel::FilterVector& state,
        typename HeadingMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(3);
    }

   
}

#endif
