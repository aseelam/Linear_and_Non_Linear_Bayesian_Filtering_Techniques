#include <generate_meas_sequence.h>
#include <mvn.h>

namespace filtering_utils
{
    Eigen::MatrixXd GenerateMeasSequence::get_meas_sequence() const
    {
        auto N = true_state_seq.cols() - 1; // Number of true states.
        auto m = meas_model->dim();         // Dimension of the measurement vector.

        const auto &meas_noise_distribution = MVN(Eigen::VectorXd::Zero(m), meas_model->get_meas_noise_covariance());

        Eigen::MatrixXd Y(m, N);
        for (int i = 1; i < N + 1; ++i)
        {
            Y.col(i-1) = meas_model->get_measurement(true_state_seq.col(i)) + meas_noise_distribution.rnd();
        }
        return Y;
    }
} // namespace filtering_utils