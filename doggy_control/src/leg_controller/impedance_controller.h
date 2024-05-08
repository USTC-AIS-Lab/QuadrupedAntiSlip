#ifndef _IMPEDANCE_CONTROLLER_H
#define _IMPEDANCE_CONTROLLER_H

#include "dog_state.h"
#include <Eigen/Dense>
#include "Utils.h"

class ImpedanceController
{
public:
    ImpedanceController();
    Eigen::Matrix<double, 3, NUM_LEG> compute_impedance_force(DogState &state);

private:
    double kp;
    double kd;
    double f_max;

};


#endif
