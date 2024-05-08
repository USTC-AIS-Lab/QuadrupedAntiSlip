#ifndef _SWING_LEG_CONTROLLER_H_
#define _SWING_LEG_CONTROLLER_H_

#include "dog_params.h"
#include "dog_state.h"

class SwingLegController
{
public:
    SwingLegController();
    Eigen::Matrix<double, 3, NUM_LEG> compute_kin_froce(DogState &state);

private:
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kp_foot; // for foot PD control
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kd_foot;
    double kp_foot_max;
    double kd_foot_max;
    Eigen::Matrix<double, 3, NUM_LEG> torques_gravity;
    void reset();
    void reset_from_rosparam();

};






#endif