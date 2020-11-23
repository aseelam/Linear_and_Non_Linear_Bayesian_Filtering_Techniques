#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Core>

namespace motion_model
{
    class motion_model_base
    {
    public:
        /**
         * @brief Returns the dimension of the assumed state vector.
         * 
         * @return unsigned int 
         */
        virtual unsigned int dim() const = 0;
        /**
         * @brief Get the motion transition/jacobian matrix object
         * 
         * @return Eigen::MatrixXd 
         */
        virtual Eigen::MatrixXd get_motion_transition_matrix(const Eigen::VectorXd& x) const =0;
        /**
         * @brief Get the motion noise covariance object
         * 
         * @return Eigen::MatrixXd 
         */
        virtual Eigen::MatrixXd get_motion_noise_covariance() const =0;
        /**
         * @brief Get the state prediction object
         * 
         * @param x : Current state.
         * @return Eigen::VectorXd 
         */
        virtual Eigen::VectorXd get_state_prediction(const Eigen::VectorXd &x) const = 0;
    };
} // namespace motion_model

#endif