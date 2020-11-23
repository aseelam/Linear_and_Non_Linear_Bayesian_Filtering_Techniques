#include <ctmodel.h>
#include <cvmodel.h>
#include <bearing_meas_model.h>
#include <generate_meas_sequence.h>
#include <generate_true_state_sequence.h>
#include <extended_kalman_filter.h>

#include <iostream>

int main()
{
    // Select motion and measurement models
    double T = 1.0;       // Sampling time
    double sigma_v = 0.25; // standard deviation of motion noise
    motion_model::cvmodel cv_motion_model(T, sigma_v);
    Eigen::VectorXd sensor_pos = Eigen::VectorXd::Zero(2);
    double sigma_y = 0.10; //standard deviation of meas noise
    meas_model::bearing_meas_model b_meas_model(sensor_pos, sigma_y);

    // Select a Prior
    Eigen::VectorXd x_0(4);
    x_0 << 1, 2, 0, 0;
    Eigen::MatrixXd P_0(x_0.size(), x_0.size());
    P_0 << 1, 0, 0.5, 0,
        0, 1, 0, 0.5,
        0.5, 0, 0.25, 0,
        0, 0.5, 0, 0.25;

    // Obtain true state sequence.
    unsigned int N = 100;
    std::cout << "Generating True State Sequence..........." << std::endl;
    const auto &X = filtering_utils::GenerateTrueStateSequence(x_0, P_0, &cv_motion_model).get_true_state_sequence(N);
    std::cout << "Generating Meas Sequence..........." << std::endl;
    const auto &Y = filtering_utils::GenerateMeasSequence(X, &b_meas_model).get_meas_sequence();
    std::cout << "Done generating sequences............" << std::endl;

    // Perform filtering
    const auto &ekf = bayesian_filters::ExtendedKalmanFilter(&cv_motion_model, &b_meas_model);
    Eigen::VectorXd x(x_0);
    Eigen::MatrixXd P(P_0);
    for (int i = 0; i < N; ++i)
    {   
        ekf.Predict(x, P);
        ekf.Update(x, P, Y.col(i));
        std::cout << "Estimate : " << x[0] << " " << x[1] << " True State : " << X.col(i + 1)[0] << " " << X.col(i + 1)[1] << std::endl;
    }
    return 0;
}