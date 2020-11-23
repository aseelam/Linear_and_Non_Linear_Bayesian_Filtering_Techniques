#include <cvmodel.h>

namespace motion_model
{
    Eigen::MatrixXd cvmodel::get_motion_transition_matrix(const Eigen::VectorXd& x) const
    {
        Eigen::MatrixXd F(dim(), dim());
        F << 1, 0, T, 0,
            0, 1, 0, T,
            0, 0, 1, 0,
            0, 0, 0, 1;
        return F;
    }
    Eigen::MatrixXd cvmodel::get_motion_noise_covariance() const
    {
        Eigen::MatrixXd Q(dim(), dim());
        Q << std::pow(T, 4) / 4, 0, std::pow(T, 3) / 2, 0,
            0, std::pow(T, 4) / 4, 0, std::pow(T, 3) / 2,
            std::pow(T, 3) / 2, 0, std::pow(T, 2), 0,
            0, std::pow(T, 3) / 2, 0, std::pow(T, 2);
        Q = std::pow(sigma,2) * Q;
        return Q;
    }
    Eigen::VectorXd cvmodel::get_state_prediction(const Eigen::VectorXd& x) const {
        Eigen::VectorXd f;
        f = get_motion_transition_matrix(x) * x;
        return f;
    }
} // namespace motion_model