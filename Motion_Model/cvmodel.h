#ifndef CVMODEL_H
#define CVMODEL_H

#include <Eigen/Core>
#include <motion_model.h>

namespace motion_model
{
    class cvmodel : public motion_model_base
    {
    public:
        cvmodel(const double T, const double sigma) : T(T), sigma(sigma) {}
        /**
         * @brief Returns the dimension of the assumed state vector.
         * 
         * @return unsigned int 
         */
        unsigned int dim() const { return 4; }
        /**
         * @brief Get the motion transition matrix object
         * 
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd get_motion_transition_matrix(const Eigen::VectorXd &x) const;
        /**
         * @brief Get the motion noise covariance object
         * 
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd get_motion_noise_covariance() const;
        /**
         * @brief Get the state prediction object
         * 
         * @param x : Current state.
         * @return Eigen::VectorXd 
         */
        Eigen::VectorXd get_state_prediction(const Eigen::VectorXd &x) const;

    private:
        const double T;     // sampling time
        const double sigma; // standard deviation of motion noise
    };

} // namespace motion_model

#endif