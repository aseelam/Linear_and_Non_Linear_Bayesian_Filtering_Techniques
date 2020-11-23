#include <extended_kalman_filter.h>
#include <Eigen/QR>    
#include <iostream>

namespace bayesian_filters
{
    void ExtendedKalmanFilter::Predict(Eigen::VectorXd &x, Eigen::MatrixXd &P) const
    {
        x = motion_model->get_state_prediction(x);
        const auto &dfx = motion_model->get_motion_transition_matrix(x); // Jacobian evaluated at x.
        P = dfx * P * dfx.transpose() + motion_model->get_motion_noise_covariance();
    }
    void ExtendedKalmanFilter::Update(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::VectorXd &y) const
    {
        const auto &y_hat = meas_model->get_measurement(x);      // Predicted meas.
        const auto &dhx = meas_model->get_observation_matrix(x); // Meas model jacobian evaluated at x.
        Eigen::MatrixXd S = dhx * P * dhx.transpose() + meas_model->get_meas_noise_covariance();
        Eigen::MatrixXd Sinv;
        if (S.rows() == 1 && S.cols() == 1)
        {
            Sinv.resize(1, 1);
            Sinv << 1.0 / S(0, 0);
        }else{
            Sinv = S.inverse();
        }
        Eigen::MatrixXd K = P * dhx.transpose() * Sinv;

        x = x + K * (y - y_hat);
        Eigen::MatrixXd Kinv = K.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd temp = S * Kinv;
        P = P - K * temp;
    }
} // namespace bayesian_filters