#include <bearing_meas_model.h>
#include <iostream>

using namespace meas_model;

int main(){
    meas_model_base* bp;
    Eigen::VectorXd sensor_pos(2);
    sensor_pos << 0,0;
    bearing_meas_model bm(sensor_pos,2.0);
    bp = &bm;
    Eigen::VectorXd state(4);
    state << 3, 4, 2, 1;
    std::cout << bp->get_measurement(state) * 180.0 / M_PI << std::endl;
    std::cout << bp->get_observation_matrix(state) << std::endl;
    return 0;
}