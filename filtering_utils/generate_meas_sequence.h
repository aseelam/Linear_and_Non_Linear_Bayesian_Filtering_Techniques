#ifndef GENERATE_MEAS_SEQUENCE_H
#define GENERATE_MEAS_SEQUENCE_H

#include <Eigen/Core>
#include <meas_model.h>

namespace filtering_utils
{
    class GenerateMeasSequence
    {
    public:
        GenerateMeasSequence(const Eigen::MatrixXd &true_state_seq, const meas_model::meas_model_base *meas_model) : true_state_seq(true_state_seq), meas_model(meas_model) {}
        Eigen::MatrixXd get_meas_sequence() const;
    private:
        const Eigen::MatrixXd true_state_seq;
        const meas_model::meas_model_base *meas_model;
    };
} // namespace filtering_utils

#endif