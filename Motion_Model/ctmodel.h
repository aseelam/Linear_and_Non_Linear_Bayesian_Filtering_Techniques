#ifndef CTMODEL_H
#define CTMODEL_H

#include <Eigen/Core>
#include <motion_model.h>

namespace motion_model
{
    class ctmodel : public motion_model_base
    {
    public:
        ctmodel(const double T, const double sigma_v,const double sigma_omega) : T(T), sigma_v(sigma_v), sigma_omega(sigma_omega) {}
        /**
         * @brief Returns the dimension of the assumed state vector.
         * 
         * @return unsigned int 
         */
        unsigned int dim() const{return 5;}
        /**
         * @brief Get the motion jacobian matrix object
         * 
         * @param x : Current state
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd get_motion_transition_matrix(const Eigen::VectorXd& x) const;
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
        Eigen::VectorXd get_state_prediction(const Eigen::VectorXd& x) const;

    private:
        const double T; // sampling time
        const double sigma_v; // standard deviation of motion noise added to polar velocity
        const double sigma_omega; // standard deviation of motion noise added to turn rate
    };

} // namespace motion_model

#endif