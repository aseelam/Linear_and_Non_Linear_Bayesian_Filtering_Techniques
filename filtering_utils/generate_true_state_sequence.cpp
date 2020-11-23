#include <generate_true_state_sequence.h>
#include <mvn.h>

namespace filtering_utils
{
    Eigen::MatrixXd GenerateTrueStateSequence::get_true_state_sequence(const unsigned int N) const
    {
        auto n = x_0.size();
        Eigen::MatrixXd X(n, N + 1);
        const auto &prior_distribution = MVN(x_0, P_0);
        const auto &motion_noise_distribution = MVN(Eigen::VectorXd::Zero(n), motion_model->get_motion_noise_covariance());
        X.col(0) = prior_distribution.rnd(); // Prior state.
        for (int i = 0; i < N; ++i)
        {
            X.col(i + 1) = motion_model->get_state_prediction(X.col(i)) + motion_noise_distribution.rnd();
        }
        return X;
    }
} // namespace filtering_utils