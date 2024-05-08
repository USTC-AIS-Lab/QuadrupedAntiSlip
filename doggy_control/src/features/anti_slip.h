#ifndef _ANTI_SLIP_H_
#define _ANTI_SLIP_H_

#include "dog_state.h"
#include "SlipDetection.h"
#include "gait/probe_gait.h"
#include "impedance_controller.h"




class AntiSlip
{
public:
    AntiSlip(GaitPtr &gait_ptr);
    void slip_detect(DogState &state);
    Eigen::Matrix<double, 3, NUM_LEG> impedance_control (DogState &state);
    Eigen::Matrix<double, NUM_DOF, 1> torque_modify(Eigen::Matrix<double, NUM_DOF, 1> joint_torques_raw, DogState &state);
    GaitPtr get_probe_gait_ptr(){return std::make_unique<ProbeGait>();}
    void friction_probe(DogState &state);
private:
    SlipDetector slip_detector_;
    GaitPtr &gait_ptr_;
    ImpedanceController impedance_controller_;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_imp_set_;
    bool impedance_activated[4];
    Eigen::Vector3d probe_force_; // in robot frame.
    bool probing_;
};



#endif
