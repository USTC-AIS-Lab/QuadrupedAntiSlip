#include "dog_state.h"
#include "Utils.h"

void AttitudeState::reset()
{
    com_quat.setIdentity();
    quat_vio_offset.setIdentity();
    com_rot_mat.setZero();
    com_rot_mat_z.setZero();
    com_euler.setZero();
    com_ang_vel.setZero();
    com_acc_r.setZero();
    imu_acc.setZero();
    com_ang_vel_r.setZero();
    foot_force.setZero();
    joint_pos_q.setZero();
    joint_vel_q.setZero();
    joint_force_q.setZero();
    foot_pos_r.setZero();
    foot_pos.setZero();
    foot_pos_abs.setZero();
    foot_vel_r.setZero();
    foot_vel.setZero();
    foot_vel_filtered.setZero();
    j_foot_r.setIdentity();
    com_pos_estimated.setZero();
    com_lin_vel_estimated.setZero();
    hip_pos_abs.setZero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        is_contact[i] = false;
        early_contact[i] = false;
    }
    com_pos.setZero();
    com_lin_vel.setZero();
    robot_mass = ROBOT_MASS;
    trunk_inertia << TRUNK_INERTIA_X, 0.0, 0.0,
        0.0, TRUNK_INERTIA_Y, 0.0,
        0.0, 0.0, TRUNK_INERTIA_Z;
    foot_force_offset.setZero();
    hip_pos_r << DEFAULT_FOOT_POS_X_ABS, DEFAULT_FOOT_POS_X_ABS, -DEFAULT_FOOT_POS_X_ABS, -DEFAULT_FOOT_POS_X_ABS,
        DEFAULT_FOOT_POS_Y_ABS, -DEFAULT_FOOT_POS_Y_ABS, DEFAULT_FOOT_POS_Y_ABS, -DEFAULT_FOOT_POS_Y_ABS,
        0.0, 0.0, 0.0, 0.0;
    debug_value.resize(20, 0.0);
    com_rot_mat_r_to_abs.setIdentity();
    last_foot_contact.setZero();
    hip_pos.setZero();
    estimated_terrain_euler.setZero();
    terrain_angle_filter = 0;
    support_polygon.setZero();
    virtual_support_polygon.setZero();
    com_pos_four_cols.setZero();
    pos_vio_offset.setZero();
    reset_from_rosparam();
}

void AttitudeState::reset_from_rosparam()
{
    ros::NodeHandle _nh;
    _nh.param("robot_mass", robot_mass, ROBOT_MASS);
    double trunk_inertia_x, trunk_inertia_y, trunk_inertia_z;
    _nh.param("trunk_inertia_x", trunk_inertia_x, TRUNK_INERTIA_X);
    _nh.param("trunk_inertia_y", trunk_inertia_y, TRUNK_INERTIA_Y);
    _nh.param("trunk_inertia_z", trunk_inertia_z, TRUNK_INERTIA_Z);
    trunk_inertia << trunk_inertia_x, 0.0, 0.0,
        0.0, trunk_inertia_y, 0.0,
        0.0, 0.0, trunk_inertia_z;
    double foot_force_offset_fl = 0;
    double foot_force_offset_fr = 0;
    double foot_force_offset_rl = 0;
    double foot_force_offset_rr = 0;
    _nh.param("foot_force_offset_fl", foot_force_offset_fl, 0.0);
    _nh.param("foot_force_offset_fr", foot_force_offset_fr, 0.0);
    _nh.param("foot_force_offset_rl", foot_force_offset_rl, 0.0);
    _nh.param("foot_force_offset_rr", foot_force_offset_rr, 0.0);
    foot_force_offset = Eigen::Vector4d(foot_force_offset_fl, foot_force_offset_fr, foot_force_offset_rl, foot_force_offset_rr);
    double default_foot_pos_x_abs, default_foot_pos_y_abs;
    _nh.param("default_foot_pos_x_abs", default_foot_pos_x_abs, DEFAULT_FOOT_POS_X_ABS);
    _nh.param("default_foot_pos_y_abs", default_foot_pos_y_abs, DEFAULT_FOOT_POS_Y_ABS);
    hip_pos_r << default_foot_pos_x_abs, default_foot_pos_x_abs, -default_foot_pos_x_abs, -default_foot_pos_x_abs,
        default_foot_pos_y_abs, -default_foot_pos_y_abs, default_foot_pos_y_abs, -default_foot_pos_y_abs,
        0.0, 0.0, 0.0, 0.0;
    _nh.getParam("terrain_angle_filter", terrain_angle_filter);
    
}

void AttitudeState::compute_support_polygon(Eigen::Vector4d leg_weights)
{
    // std::cout << "leg_weights: " << leg_weights.transpose() << std::endl;
    for(int i = 0; i < NUM_LEG; i++)
    {
        if(is_contact[i])
        {
            support_polygon.block<3, 1>(0, i) = foot_pos.block<3, 1>(0, i);
        }
        else
        {
            support_polygon.block<3, 1>(0, i).setZero();
        }
        if(leg_weights(i) > 0)
        {
            virtual_support_polygon.block<3, 1>(0, i) = com_pos + (foot_pos.block<3, 1>(0, i) - com_pos) * leg_weights(i);
        }
        else
        {
            virtual_support_polygon.block<3, 1>(0, i).setZero();
        }
        support_polygon.block<3, 1>(0, i)(2) = 0;
        virtual_support_polygon.block<3, 1>(0, i)(2) = 0;
    }
}

void CtrlState::reset()
{
    joint_torques_set.setZero();
    foot_forces_kin_set.setZero();
    foot_forces_grf_set.setZero();
    com_euler_set.setZero();
    com_pos_set.setZero();
    com_ang_vel_r_set.setZero();
    com_lin_vel_r_set.setZero();
    foot_pos_r_set.setZero();
    foot_pos_set.setZero();
    foot_vel_r_set.setZero();
    foot_target_set.setZero();
    com_ang_vel_r_set.setZero();
    com_lin_vel_set.setZero();
    com_lin_vel_actuated_set.setZero();
    com_ang_vel_actuated_set.setZero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        plan_contact[i] = false;
    }
    max_torque = MAX_TORQUE;
    foot_force_filter = FOOT_FORCE_FILTER;
    foot_contact_threshold = FOOT_CONTACT_THRESHOLD;
    safe_power_level = 0;
    safety_halt = false;
    tilt_threshold = 0.3;
    doggy_cmd_received = false;
    sensor_received = false;
    max_lin_acc = 0;
    max_ang_acc = 0;
    com_offset_abs.setZero();
    com_set.setZero();
    yaw_offset_set = 0;
    doggy_cmd.cmd_code = doggy_cmd.CMD_EMPTY;
    stand_status = 0;
    udp_online = true;
    foot_forces_imp_set.setZero();
    reset_from_rosparam();
}

void CtrlState::reset_from_rosparam()
{
    ros::NodeHandle _nh;
    _nh.param("max_torque", max_torque, MAX_TORQUE);
    _nh.param("foot_force_filter", foot_force_filter, FOOT_FORCE_FILTER);
    _nh.param("foot_contact_threshold", foot_contact_threshold, FOOT_CONTACT_THRESHOLD);
    _nh.param("safe_power_level", safe_power_level, SAFE_POWER_LEVEL);
    _nh.param("tilt_threshold", tilt_threshold, TILT_THRESHOLD);
    _nh.param("default_body_height", default_body_height, DEFAULT_BODY_HEIGHT);
    _nh.getParam("max_lin_acc", max_lin_acc); 
    _nh.getParam("max_ang_acc", max_ang_acc); 
}

void DogState::reset()
{
    attitude.reset();
    ctrl.reset();
}

void DogState::state_process()
{
    Eigen::Matrix<double, 3, NUM_LEG> com_offset_abs_four_cols;
    for(int i = 0; i < NUM_LEG; ++i)
    {
        attitude.com_pos_four_cols.block<3, 1>(0, i) = attitude.com_pos;
        com_offset_abs_four_cols.block<3, 1>(0, i) = ctrl.com_offset_abs;
    }
    attitude.com_rot_mat = attitude.com_quat.toRotationMatrix();
    attitude.com_euler = Utils::quat_to_euler(attitude.com_quat);
    double yaw_angle = attitude.com_euler[2];
    if (!ctrl.sensor_received)
    {
        ctrl.com_euler_set(2) = yaw_angle;
    }
    attitude.com_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());
    attitude.com_rot_mat_r_to_abs = attitude.com_rot_mat_z.transpose() * attitude.com_rot_mat;
    attitude.com_acc_r = attitude.imu_acc + attitude.com_rot_mat.transpose() * Eigen::Vector3d(0, 0, -GRAVITY_ACCELERATION);

    // FL, FR, RL, RR
    for (int i = 0; i < NUM_LEG; ++i)
    {
        attitude.foot_pos_r.block<3, 1>(0, i) = a1_kin.fk(attitude.joint_pos_q.segment<3>(3 * i), i);
        attitude.j_foot_r.block<3, 3>(3 * i, 3 * i) = a1_kin.jac(attitude.joint_pos_q.segment<3>(3 * i), i);
        Eigen::Matrix3d tmp_mtx = attitude.j_foot_r.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = attitude.joint_vel_q.segment<3>(3 * i);
        attitude.foot_vel_r.block<3, 1>(0, i) = tmp_mtx * tmp_vec;
    }
    attitude.hip_pos_abs = attitude.com_rot_mat_r_to_abs * attitude.hip_pos_r; // - com_offset_abs_four_cols;
    attitude.hip_pos = attitude.com_rot_mat * attitude.hip_pos_r + attitude.com_pos_four_cols;

    attitude.foot_pos_abs = attitude.com_rot_mat_r_to_abs * attitude.foot_pos_r;
    attitude.foot_pos  = attitude.com_rot_mat * attitude.foot_pos_r + attitude.com_pos_four_cols;
    attitude.com_ang_vel = attitude.com_rot_mat * attitude.com_ang_vel_r;
    attitude.euler_to_ang_vel_mapping = Utils::euler_to_angular_velocity_mapping(attitude.com_euler);

    ctrl.com_set = ctrl.com_pos_set + attitude.com_rot_mat_z * ctrl.com_offset_abs;
    
    attitude.com_ang_vel_r_filtered = 0.95 * attitude.com_ang_vel_r_filtered + 0.05 * attitude.com_ang_vel_r;
    attitude.com_ang_vel_filtered =  attitude.com_rot_mat * attitude.com_ang_vel_r_filtered;
    attitude.com_lin_vel_r_filtered = 0.95 * attitude.com_lin_vel_r_filtered + 0.05 * attitude.com_lin_vel_r;
    attitude.com_lin_vel_filtered = attitude.com_rot_mat * attitude.com_lin_vel_r_filtered;
    
    for (int i = 0; i < NUM_LEG; ++i)
    {
        attitude.foot_vel.block<3, 1>(0, i) = attitude.com_rot_mat * attitude.foot_vel_r.block<3, 1>(0, i) + attitude.com_lin_vel + Utils::skew(attitude.com_ang_vel) * attitude.com_rot_mat * attitude.foot_pos_r.block<3, 1>(0, i);
        attitude.foot_vel_filtered.block<3, 1>(0, i) = 0.9 * attitude.foot_vel_filtered.block<3, 1>(0, i) + 0.1 * attitude.foot_vel.block<3, 1>(0, i);
    }

    contact_estimate();
    terrain_angle_estimate(); 

    
}

void DogState::terrain_angle_estimate()
{
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if(attitude.is_contact[i])
        {
            attitude.last_foot_contact.block<3, 1>(0, i) = attitude.foot_pos.block<3, 1>(0, i);
        }
    }
    attitude.estimated_terrain_euler = attitude.terrain_angle_filter * attitude.estimated_terrain_euler + (1.0 - attitude.terrain_angle_filter) * Utils::calculate_surface_euler(attitude.last_foot_contact, attitude.com_euler(2));


    // std::cout << "estimated_terrain_euler" << std::endl;
    // std::cout << attitude.estimated_terrain_euler.transpose() << std::endl;
}



bool DogState::contact(int i)
{
    //return attitude.is_contact[i] || ctrl.plan_contact[i];
    return ctrl.plan_contact[i] || attitude.early_contact[i];
}

void DogState::contact_estimate()
{
    bool is_prone = true;
    for (int i = 0; i < 4; ++i)
    {
        if (attitude.foot_pos_r(2, i) < PRONE_THRESHOLD_Z)
        {
            is_prone = false;
        }
    }
    for (int i = 0; i < 4; ++i)
    {
        attitude.is_contact[i] = is_prone || attitude.foot_force[i] > ctrl.foot_contact_threshold;
        // bool new_contact = is_prone || attitude.foot_force[i] > ctrl.foot_contact_threshold;
        // if(new_contact && ! attitude.is_contact[i])
        // {
        //     attitude.last_foot_contact.block<3, 1>(0, i) = attitude.foot_pos.block<3, 1>(0, i);
        // }
        // attitude.is_contact[i] = new_contact;
    }
}