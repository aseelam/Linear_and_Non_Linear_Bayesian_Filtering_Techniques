#ifndef MEAS_MODEL_H
#define MEAS_MODEL_H

#include <Eigen/Core>

namespace meas_model
{
    class meas_model_base
    {
    public:
        /**
         * @brief Returns the dimension of the measurement vector.
         * 
         * @return unsigned int 
         */
        virtual unsigned int dim() const = 0;
        /**
         * @brief Get the measurement model/observation matrix object
         * 
         * @param x : Current state.
         * @return Eigen::MatrixXd 
         */
        virtual Eigen::MatrixXd get_observation_matrix(const Eigen::VectorXd &x) const = 0;
        /**
         * @brief Get the measurement noise covariance object
         * 
         * @return Eigen::MatrixXd 
         */
        virtual Eigen::MatrixXd get_meas_noise_covariance() const = 0;
        /**
         * @brief Get the measurement from state
         * 
         * @param x : Current state.
         * @return Eigen::VectorXd 
         */
        virtual Eigen::VectorXd get_measurement(const Eigen::VectorXd &x) const = 0;
    };
} // namespace meas_model

#endif