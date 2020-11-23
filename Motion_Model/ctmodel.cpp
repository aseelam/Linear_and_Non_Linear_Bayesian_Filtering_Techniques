#include <ctmodel.h>

namespace motion_model
{
    Eigen::MatrixXd ctmodel::get_motion_transition_matrix(const Eigen::VectorXd &x) const
    {
        Eigen::MatrixXd F(dim(), dim());
        F << 1, 0, T * cos(x(4)), -T * x(3) * sin(x(4)), 0,
            0, 1, T * sin(x(4)), T * x(3) * cos(x(4)), 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, T,
            0, 0, 0, 0, 1;
        return F;
    }
    Eigen::MatrixXd ctmodel::get_motion_noise_covariance() const
    {
        Eigen::MatrixXd Q(dim(), dim());
        Eigen::MatrixXd G(dim(), 2);
        G << 0, 0,
            0, 0,
            1, 0,
            0, 0,
            0, 1;
        Eigen::MatrixXd sigma_square(2, 2);
        sigma_square << std::pow(sigma_v, 2), 0,
            0, std::pow(sigma_omega, 2);
        Q = G * sigma_square * G.transpose();
        return Q;
    }
    Eigen::VectorXd ctmodel::get_state_prediction(const Eigen::VectorXd &x) const
    {
        Eigen::VectorXd transition(dim());
        transition << T * x(3) * cos(x(4)), T * x(3) * sin(x(4)), 0, T * x(5), 0;
        return x + transition;
    }
} // namespace motion_model