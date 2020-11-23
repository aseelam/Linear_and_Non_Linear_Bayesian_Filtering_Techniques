#include <cvmodel.h>
#include <ctmodel.h>
#include <iostream>

using namespace motion_model;

int main(){
    motion_model_base* bp;
    cvmodel cv(1.0,2.0);
    bp = &cv;
    std::cout << bp->get_motion_noise_covariance() << std::endl;
    Eigen::VectorXd x(4);
    x << 1,2,3,2;
    std::cout << bp->get_motion_transition_matrix(x) << std::endl;
    std::cout << bp->get_state_prediction(x) << std::endl;
    return 0;
}