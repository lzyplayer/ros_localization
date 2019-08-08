#ifndef MEASUREMENTS_HEADING_MEASUREMENT_HPP
#define MEASUREMENTS_HEADING_MEASUREMENT_HPP

#include "ukf_core/sensor_measurement_base.hpp"
#include "ukf_core/models/smodel_ukf2d.hpp"


#define PI           3.14159265358979323846  /* pi */
using Eigen::MatrixXd;
using Eigen::VectorXd;
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
    struct is_support<HeadingMeasurement, SModel> : std::true_type {};





template<>
    void getMeasurementPrediction<HeadingMeasurement,SModel>(
        const typename SModel::ModelParameter& parameter,
        const Eigen::MatrixXd& Xsig_pred,
        Eigen::MatrixXd& Zsig,
        typename HeadingMeasurement::MeasurementVector& measurement_pred
    )
    {
        int n_sig = Xsig_pred.cols();

        Zsig = Xsig_pred.block(4, 0, 1, n_sig);
        measurement_pred.setZero();


        // for (int i = 0; i < n_sig; i++) 
        // {
        //     measurement_pred = measurement_pred + parameter.weights_m_(i) * Z_sig.col(i);//等号左右类型相同
        // }
        // for (int i = 0; i < n_sig; i++) 
        // {
        //     Zsig.col(i) = Z_sig.col(i) - measurement_pred;///
        //     double a=Zsig(i);
        //     if (a > PI) {Zsig(i) = Zsig(i) - 2. * PI;}
		//     else if (a < -PI) {Zsig(i) = Zsig(i) + 2. * PI;}
        // }
    }


    template<>
    void getMeasurementPub<HeadingMeasurement, SModel>(
        const typename SModel::ModelParameter& parameter,
        const typename SModel::FilterVector& state,
        typename HeadingMeasurement::MeasurementVector& measurement
    )
    {
        measurement << state(4);//车轴朝向
    }
}

#endif
