#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <motion_model.h>
#include <meas_model.h>
#include <Eigen/Core>

namespace bayesian_filters
{
    class ExtendedKalmanFilter
    {
    public:
        ExtendedKalmanFilter(const motion_model::motion_model_base *motion_model, const meas_model::meas_model_base *meas_model) : motion_model(motion_model), meas_model(meas_model) {}
        void Predict(Eigen::VectorXd& x, Eigen::MatrixXd& P) const;
        void Update(Eigen::VectorXd& x, Eigen::MatrixXd& P, const Eigen::VectorXd& y) const;

    private:
        const motion_model::motion_model_base *motion_model;
        const meas_model::meas_model_base *meas_model;
    };
} // namespace bayesian_filters

#endif