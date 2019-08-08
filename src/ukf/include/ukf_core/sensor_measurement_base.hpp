#ifndef SENSOR_MEASUREMENT_BASE
#define SENSOR_MEASUREMENT_BASE

#include "ukf_core/kalman_filter_model_base.hpp"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

namespace SensorMeasurementConverter////////////////////////////////////////////////////////////////////
{
    template<typename TMeasurement, typename TModel>
    struct is_support : std::false_type {};
}

template<int NMeasurement>
class SensorMeasurementBase
{
public:
    static const int NumberMeasurement = NMeasurement;

    typedef Eigen::Matrix<double, NMeasurement, 1> MeasurementVector;
    typedef Eigen::Matrix<int, NMeasurement, 1> MeasurementFlagVector;
    typedef Eigen::Matrix<double, NMeasurement, NMeasurement> MeasurementMatrix;

    SensorMeasurementBase()
    {
        measurement_.setZero();
        covariance_.setZero();
    }

    void setMeasurement(const MeasurementVector& measurement, const MeasurementMatrix& covariance, double timestamp)
    {
        boost::mutex::scoped_lock lock(mutex_);//////////////////////////////////////////////////////////////////////////////////////////////////////////
        measurement_ = measurement;
        covariance_ = covariance;
        timestamp_ = timestamp;
    }

    void changeMeasurement(const MeasurementVector& measurement)
    {
        boost::mutex::scoped_lock lock(mutex_);//////////////////////////////////////////////////////////////////////////////////////////////////////////
        measurement_ += measurement;
    }

    void getMeasurement(MeasurementVector& measurement, MeasurementMatrix& covariance, double& timestamp)
    {
        boost::mutex::scoped_lock lock(mutex_);
        measurement = measurement_;
        covariance = covariance_;
        timestamp = timestamp_;
    }

    double getTime()
    {
        return timestamp_;
    }

    virtual void getAngleFlag(
        MeasurementFlagVector& angle_flag
    ) const = 0;

private:
  //成员变量
    boost::mutex mutex_;
    double timestamp_;
    MeasurementVector measurement_;
    MeasurementMatrix covariance_;
};

namespace SensorMeasurementConverter
{
    template<typename TMeasurement, typename TModel>
    void getMeasurementPub(
        const typename TModel::ModelParameter& parameter,
        const typename TModel::FilterVector& state,
        typename TMeasurement::MeasurementVector& measurement
    );

    template<typename TMeasurement, typename TModel>
    void getMeasurementPrediction(
        const typename TModel::ModelParameter& parameter,
        const Eigen::MatrixXd& Xsig_pred,
        Eigen::MatrixXd& Zsig,
        typename TMeasurement::MeasurementVector& measurement_pred
    );
}

#endif