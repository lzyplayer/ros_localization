#ifndef MEASUREMENTS_WHEEL_MEASUREMENT_HPP
#define MEASUREMENTS_WHEEL_MEASUREMENT_HPP

// #include <string>
// #include <iostream>

#include "ekf_core/sensor_measurement_base.hpp"
#include "ekf_core/models/ctra.hpp"
#include "ekf_core/models/akerman.hpp"
#include "ekf_core/models/bicycle.hpp"

class WheelMeasurement : public SensorMeasurementBase<4>
{
public:
    void getAngleFlag(
        MeasurementFlagVector& angle_flag
    ) const override final
    {
        angle_flag << 0,0,0,0;
    }
};

namespace SensorMeasurementConverter
{
    template<>
    struct is_support<WheelMeasurement, AKERMANModel> : std::true_type {};

    template<>
    struct is_support<WheelMeasurement, CTRAModel> : std::true_type {};

    template<>
    struct is_support<WheelMeasurement, BICYCLEModel> : std::true_type {};

    template<>
    void getMeasurementJacobian<WheelMeasurement, AKERMANModel>(
        const typename AKERMANModel::ModelParameter& parameter,
        const typename AKERMANModel::FilterVector& state,
        Eigen::Matrix<double, WheelMeasurement::NumberMeasurement, AKERMANModel::NumberState>& H
    )
    {
        double v=state(3);
        double w=state(4);
        //double R=v/w;
        double W=parameter.W;
        double L=parameter.L;
        double fml=sqrt(4*w*w*L*L+(2*v-w*W)*(2*v-w*W));
        double fmr=sqrt(4*w*w*L*L+(2*v+w*W)*(2*v+w*W));
        H << 0, 0, 0, (2*v-w*W)/fml, (4*w*L*L-W*(2*v-w*W))/(2*fml), 0,
             0, 0, 0, (2*v+w*W)/fmr, (4*w*L*L+W*(2*v+w*W))/(2*fmr), 0,
             0, 0, 0, 1, -W/2, 0,
             0, 0, 0, 1, W/2, 0;
        H=H/parameter.R_w;
        std::cout<<"v:"<< v <<std::endl;
        std::cout<<"w:"<< w <<std::endl;
        std::cout<<"fml:"<< fml <<std::endl;
        std::cout<<"fmr:"<< fmr <<std::endl;
        std::cout<<"轮速计H:"<< H <<std::endl;
    }

    template<>
    void getMeasurementPrediction<WheelMeasurement, AKERMANModel>(
        const typename AKERMANModel::ModelParameter& parameter,
        const typename AKERMANModel::FilterVector& state,
        typename WheelMeasurement::MeasurementVector& measurement
    )
    {
        double v=state(3);
        double w=state(4);
        //double R=v/w;
        double W=parameter.W;
        double L=parameter.L;
        // double fl=atan2(parameter.L, R-W/2);//R有正负,fl也有
        // double fr=atan2(parameter.L, R+W/2);
        //measurement << (v-0.5*w*W)*cos(fl)+w*L*sin(fl), (v+0.5*w*W)*cos(fr)+w*L*sin(fr), v-0.5*w*W, v+0.5*w*W;// fl,fr,rl,rr
        measurement << sqrt(4*w*w*L*L+(2*v-w*W)*(2*v-w*W))/2, sqrt(4*w*w*L*L+(2*v+w*W)*(2*v+w*W))/2, v-w*W/2, v+w*W/2;// fl,fr,rl,rr
        measurement = measurement/parameter.R_w;
    } 


    template<>
    void getMeasurementJacobian<WheelMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        Eigen::Matrix<double, WheelMeasurement::NumberMeasurement, CTRAModel::NumberState>& H
    )
    {
        H << 0, 0, 0, 1, 0, 0;
    }

    template<>
    void getMeasurementPrediction<WheelMeasurement, CTRAModel>(
        const typename CTRAModel::ModelParameter& parameter,
        const typename CTRAModel::FilterVector& state,
        typename WheelMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(3);////////////////////////////////从估计状态输出一些直观有用的输出///////////////////////////提供输出当前估计速度的接口
    }



    // template<>
    // void getMeasurementJacobian<WheelMeasurement, BICYCLEModel>(
    //     const typename BICYCLEModel::ModelParameter& parameter,
    //     const typename BICYCLEModel::FilterVector& state,
    //     Eigen::Matrix<double, WheelMeasurement::NumberMeasurement, BICYCLEModel::NumberState>& H
    // )
    // {
    //     H << 0, 0, 0, 0, 1, 0;
    // }

    // template<>
    // void getMeasurementPrediction<WheelMeasurement, BICYCLEModel>(
    //     const typename BICYCLEModel::ModelParameter& parameter,
    //     const typename BICYCLEModel::FilterVector& state,
    //     typename WheelMeasurement::MeasurementVector& measurement
    // )
    // {
    //     measurement << state(4);
    // }

}//

#endif
