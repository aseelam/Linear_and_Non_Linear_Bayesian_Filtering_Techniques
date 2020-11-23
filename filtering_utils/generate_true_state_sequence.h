#ifndef GENERATE_TRUE_STATE_SEQUENCE_H
#define GENERATE_TRUE_STATE_SEQUENCE_H

#include <Eigen/Core>
#include <motion_model.h>

namespace filtering_utils
{
    class GenerateTrueStateSequence
    {
    public:
        GenerateTrueStateSequence(const Eigen::VectorXd &x_0, const Eigen::MatrixXd &P_0, const motion_model::motion_model_base *motion_model) : x_0(x_0), P_0(P_0), motion_model(motion_model) {}

        Eigen::MatrixXd get_true_state_sequence(const unsigned int N) const;
    private:
        const Eigen::VectorXd x_0;                           // Prior mean.
        const Eigen::MatrixXd P_0;                           // Prior covariance.
        const motion_model::motion_model_base *motion_model; // Pointer to the corresponding motion model.
    };
} // namespace filtering_utils

#endif