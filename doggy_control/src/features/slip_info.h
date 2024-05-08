#ifndef _SLIP_INFO_H_
#define _SLIP_INFO_H_

#include "dog_state.h"



class SlipInfo
{
public:
    SlipInfo()
    {
        foot_slip_spot.setZero();
        slip_state.setZero();
        is_probing = false;
        probe_force.setZero();
        probe_leg = 1;
        probe_slid_detected = false;
    }
    Eigen::Matrix<double, 3, NUM_LEG> foot_slip_spot;
    Eigen::Vector4i slip_state;
    bool is_probing;
    Eigen::Vector3d probe_force;
    int probe_leg;
    bool probe_slid_detected;
};

#endif