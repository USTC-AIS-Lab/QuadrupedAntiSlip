#include "swing_leg_controller.h"


SwingLegController::SwingLegController()
{
    reset();
}

void SwingLegController::reset()
{
    kp_foot << KP_FOOT_X, KP_FOOT_X, KP_FOOT_X, KP_FOOT_X,
        KP_FOOT_Y, KP_FOOT_Y, KP_FOOT_Y, KP_FOOT_Y,
        KP_FOOT_Z, KP_FOOT_Z, KP_FOOT_Z, KP_FOOT_Z;
    kd_foot << KD_FOOT_X, KD_FOOT_X, KD_FOOT_X, KD_FOOT_X,
        KD_FOOT_Y, KD_FOOT_Y, KD_FOOT_Y, KD_FOOT_Y,
        KD_FOOT_Z, KD_FOOT_Z, KD_FOOT_Z, KD_FOOT_Z;
    kp_foot_max = 0;
    kd_foot_max = 0;
    torques_gravity << 0, 0, 0, 0,
        0, 0, 0, 0,
        GRAVITY_COMPENSATION, GRAVITY_COMPENSATION, GRAVITY_COMPENSATION, GRAVITY_COMPENSATION;
    reset_from_rosparam();
}

void SwingLegController::reset_from_rosparam()
{
    ros::NodeHandle _nh;
    double kp_foot_x, kp_foot_y, kp_foot_z;
    _nh.param("kp_foot_x", kp_foot_x, KP_FOOT_X);
    _nh.param("kp_foot_y", kp_foot_y, KP_FOOT_Y);
    _nh.param("kp_foot_z", kp_foot_z, KP_FOOT_Z);
    kp_foot << kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
        kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
        kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
    double kd_foot_x, kd_foot_y, kd_foot_z;
    _nh.param("kd_foot_x", kd_foot_x, KD_FOOT_X);
    _nh.param("kd_foot_y", kd_foot_y, KD_FOOT_Y);
    _nh.param("kd_foot_z", kd_foot_z, KD_FOOT_Z);
    kd_foot << kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
        kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
        kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;
    _nh.param("kp_foot_max", kp_foot_max, 0.0);
    _nh.param("kd_foot_max", kd_foot_max, 0.0);
    double gravity_compensation;
    _nh.param("gravity_compensation", gravity_compensation, GRAVITY_COMPENSATION);
    torques_gravity << 0, 0, 0, 0,
        0, 0, 0, 0,
        gravity_compensation, gravity_compensation, gravity_compensation, gravity_compensation;
}

Eigen::Matrix<double, 3, NUM_LEG> SwingLegController::compute_kin_froce(DogState &state)
{
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin_set;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_p_force;
    Eigen::Matrix<double, 3, NUM_LEG> foot_d_force;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        foot_pos_error.block<3, 1>(0, i) = state.ctrl.foot_pos_r_set.block<3, 1>(0, i) - state.attitude.foot_pos_r.block<3, 1>(0, i);
        foot_vel_error.block<3, 1>(0, i) = state.ctrl.foot_vel_r_set.block<3, 1>(0, i) - state.attitude.foot_vel_r.block<3, 1>(0, i);
        foot_p_force.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(kp_foot.block<3, 1>(0, i));
        foot_d_force.block<3, 1>(0, i) = foot_vel_error.block<3, 1>(0, i).cwiseProduct(kd_foot.block<3, 1>(0, i));
        if (foot_p_force.block<3, 1>(0, i).norm() > kp_foot_max)
        {
            foot_p_force.block<3, 1>(0, i) = foot_p_force.block<3, 1>(0, i) * kp_foot_max / foot_p_force.block<3, 1>(0, i).norm();
        }
        if (foot_d_force.block<3, 1>(0, i).norm() > kd_foot_max)
        {
            foot_d_force.block<3, 1>(0, i) = foot_d_force.block<3, 1>(0, i) * kd_foot_max / foot_d_force.block<3, 1>(0, i).norm();
        }
        foot_forces_kin_set.block<3, 1>(0, i) = foot_p_force.block<3, 1>(0, i) + foot_d_force.block<3, 1>(0, i) + state.attitude.com_rot_mat.transpose() * torques_gravity.block<3, 1>(0, i); // PD control
    }

    // state.attitude.debug_value[0] = state.ctrl.foot_pos_r_set(0, 1);
    // state.attitude.debug_value[1] = state.ctrl.foot_pos_r_set(1, 1);
    // state.attitude.debug_value[2] = state.ctrl.foot_pos_r_set(2, 1);
    // state.attitude.debug_value[3] = state.ctrl.foot_vel_r_set(0, 1);
    // state.attitude.debug_value[4] = state.ctrl.foot_vel_r_set(1, 1);
    // state.attitude.debug_value[5] = state.ctrl.foot_vel_r_set(2, 1);
    return foot_forces_kin_set;
}