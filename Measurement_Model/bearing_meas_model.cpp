#include <bearing_meas_model.h>

namespace meas_model
{
    Eigen::MatrixXd bearing_meas_model::get_observation_matrix(const Eigen::VectorXd &x) const {
        const auto& diff = x.head(2) - sensor_pos;
        double range = diff.norm();
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,x.size());
        H(0,0) = -(x(1)-sensor_pos(1))/std::pow(range,2);
        H(0,1) = (x(0)-sensor_pos(0))/std::pow(range,2);
        return H;
    }
    Eigen::MatrixXd bearing_meas_model::get_meas_noise_covariance() const {
        Eigen::MatrixXd R(1,1);
        R << std::pow(sigma,2);
        return R;
    }
    Eigen::VectorXd bearing_meas_model::get_measurement(const Eigen::VectorXd &x) const
    {
        Eigen::VectorXd y(1);
        y << atan2(x(1) - sensor_pos(1), x(0) - sensor_pos(0));
        return y;
    }
} // namespace meas_model