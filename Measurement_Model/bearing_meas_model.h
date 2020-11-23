#ifndef BEARING_MEAS_MODEL_H
#define BEARING_MEAS_MODEL_H

#include <meas_model.h>

namespace meas_model
{
    class bearing_meas_model : public meas_model_base
    {
    public:
        bearing_meas_model(const Eigen::Vector2d &sensor_pos, const double sigma) : sensor_pos(sensor_pos), sigma(sigma) {}
        /**
        * @brief Returns the dimension of the measurement vector.
        * 
        * @return unsigned int 
        */
        unsigned int dim() const { return 1; }
        /**
        * @brief Get the measurement model/observation matrix object
        * 
        * @param x : Current state
        * @return Eigen::MatrixXd 
        */
        Eigen::MatrixXd get_observation_matrix(const Eigen::VectorXd &x) const;
        /**
        * @brief Get the measurement noise covariance object
        * 
        * @return Eigen::MatrixXd 
        */
        Eigen::MatrixXd get_meas_noise_covariance() const;
        /**
        * @brief Get the bearing measurement from state x. It is assumed that the first two entries are x and y pos respectively.
        * 
        * @param x : Current state.
        * @return Eigen::VectorXd 
        */
        Eigen::VectorXd get_measurement(const Eigen::VectorXd &x) const;

    private:
        const double sigma;               // standard deviation of the measurement noise.
        const Eigen::Vector2d sensor_pos; // sensor position.
    };
} // namespace meas_model

#endif