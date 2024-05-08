#include "impedance_controller.h"

ImpedanceController::ImpedanceController()
{
    ros::NodeHandle _nh;
    _nh.param("anti_slip/impedance_kp", kp, 1200.0);
    _nh.param("anti_slip/impedance_kd", kd, 5.0);
    _nh.param("anti_slip/impedance_f_max", f_max, 100.0);
}

Eigen::Matrix<double, 3, NUM_LEG> ImpedanceController::compute_impedance_force(DogState &state)
{
    Eigen::Matrix<double, 3, NUM_LEG> impedance_force;
    for(int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Vector3d current_pos = state.attitude.foot_pos.block<3, 1>(0, i);
        Eigen::Vector3d target_pos = state.slip_info.foot_slip_spot.block<3, 1>(0, i);
        Eigen::Vector3d target_diff = target_pos - current_pos;
        target_diff(2) = 0;
        Eigen::Vector3d force_cartesian = kp * target_diff - kd * state.attitude.foot_vel_filtered.block<3, 1>(0, i);
        // Eigen::Vector3d force_cartesian = kp * target_diff - kd * state.attitude.foot_vel.block<3, 1>(0, i);
        // std::cout << force_cartesian.norm() << ", ";
        if(force_cartesian.norm() > f_max)
        {
            force_cartesian = force_cartesian * f_max / force_cartesian.norm();
        }
        // std::cout << force_cartesian.norm() << std::endl;
        impedance_force.block<3, 1>(0, i) = state.attitude.com_rot_mat.transpose() * force_cartesian;
        // std::cout << current_pos.transpose() << ", " << target_pos.transpose() << ", " << force_cartesian.transpose() << std::endl;
    }
    
    return impedance_force;
}
