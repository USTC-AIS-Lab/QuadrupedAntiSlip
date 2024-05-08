#include "dog_control.h"

DogControl::DogControl(): anti_slip_(gait_ptr_)
{
    ros::NodeHandle _nh;
    std::string controller_name;
    _nh.getParam("controller", controller_name);
    // robot_log_.init();
    gait_ptr_ = std::make_unique<WalkGait>();
    // gait_ptr_ = std::make_unique<Pee>();
    if (strcmp(controller_name.c_str(), "mpc") == 0)
    {
        controller_ptr_ = std::make_unique<MPCController>(gait_ptr_);
    }
    else if (strcmp(controller_name.c_str(), "qp") == 0)
    {
        controller_ptr_ = std::make_unique<QPController>();
    }
    else
    {
        ROS_ERROR_NAMED("dog_control", "unknown controller: %s", controller_name.c_str());
    }


}

void DogControl::compute_joint_torques(DogState &state)
{
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    joint_torques.setZero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Matrix3d jac = state.attitude.j_foot_r.block<3, 3>(3 * i, 3 * i);
        if (state.contact(i))
        {
            joint_torques.segment<3>(i * 3) = jac.transpose() * -state.ctrl.foot_forces_grf_set.block<3, 1>(0, i);
            
            if(state.slip_info.slip_state(i))
            {
                joint_torques.segment<3>(i * 3) += jac.transpose() * state.ctrl.foot_forces_imp_set.block<3, 1>(0, i);
            }
        }
        else
        {
            // swing leg PD
            joint_torques.segment<3>(i * 3) = jac.transpose() * state.ctrl.foot_forces_kin_set.block<3, 1>(0, i);
            // joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt);
        }
    }

    joint_torques = anti_slip_.torque_modify(joint_torques, state);

    for (int i = 0; i < NUM_DOF; ++i)
    {
        if (!isnan(joint_torques[i]))
            state.ctrl.joint_torques_set[i] = joint_torques[i];
    }
}

void DogControl::leg_kinematic_control(DogState &state, double dt)
{
    gait_ptr_->compute_swing_legs_kinematics(state, dt);
    state.ctrl.foot_forces_kin_set = swing_leg_controller.compute_kin_froce(state);
    state.ctrl.foot_forces_imp_set = anti_slip_.impedance_control(state);
}

void DogControl::com_offset_ctrl(DogState &state)
{
    Eigen::Vector4d leg_weights = gait_ptr_->calculate_foot_support_weight();
    state.attitude.compute_support_polygon(leg_weights);
    if(leg_weights.sum() > 0)
    {
        state.ctrl.com_offset_abs = gait_ptr_->com_offset_ratio * state.attitude.foot_pos_abs * leg_weights / leg_weights.sum(); 
    }
    else
    {
        state.ctrl.com_offset_abs = state.attitude.foot_pos_abs * Eigen::Vector4d(0.25, 0.25, 0.25, 0.25);
    }
    
    // state.ctrl.com_offset_abs = state.attitude.com_rot_mat_r_to_abs * state.attitude.hip_pos_r * leg_weights;
    state.ctrl.com_offset_abs(2) = 0;
    // std::cout << "leg_weights: " << leg_weights.transpose() << std::endl;
    // std::cout << "com_offset_abs: " << state.ctrl.com_offset_abs.transpose() << std::endl;
}

void DogControl::compute_grf(DogState &state, double dt)
{
    if (!state.ctrl.sensor_received)
    {
        return;
    }
    com_offset_ctrl(state);
    state.ctrl.foot_forces_grf_set = controller_ptr_->compute_ground_reaction_force(state, dt);
}

bool DogControl::stand_ctrl(DogState &state)
{
    static bool start = true;

    if (gait_ptr_->ready_to_detach())
    {
        if (state.ctrl.doggy_cmd.cmd_code == state.ctrl.doggy_cmd.CMD_LAY_DOWN)
        {
            state.ctrl.stand_status = 1;
            //std::cout << "laying down" << std::endl;
        }
        else if (state.ctrl.doggy_cmd.cmd_code == state.ctrl.doggy_cmd.CMD_STAND_UP)
        {
            state.ctrl.stand_status = 2;
            //std::cout << "standing up" << std::endl;
        }
    }
    if (state.ctrl.stand_status == 3)
    {
        start = true;
        return true;
    }

    
    if (state.ctrl.stand_status == 2) // standing up
    {
        if (start)
        {
            state.ctrl.com_pos_set = state.attitude.com_pos;
            state.ctrl.com_euler_set.setZero();
            state.ctrl.com_euler_set(2) = state.attitude.com_euler(2);
            //state.ctrl.com_pos_set(2) = 0.05;
            start = false;
        }
        if (state.ctrl.com_pos_set(2) < gait_ptr_->get_body_height(state))
        {
            state.ctrl.com_pos_set(2) += 0.0001;
        }
        else
        {
            state.ctrl.stand_status = 3;
        }
    }
    else if (state.ctrl.stand_status == 1) // laying down
    {
        if (start)
        {
            state.ctrl.com_pos_set = state.attitude.com_pos;
            state.ctrl.com_euler_set.setZero();
            state.ctrl.com_euler_set(2) = state.attitude.com_euler(2);
            // state.ctrl.com_pos_set(2) = 0.05;
            start = false;
        }
        if (state.attitude.com_pos(2) > 0.08 && state.ctrl.com_pos_set(2) > -0.2)
        {
            state.ctrl.com_pos_set(2)-= 0.0002;
        }
        else
        {
            state.ctrl.stand_status = 0;
        }
    }

    return false;
}

void DogControl::robot_fall_detect(DogState &state)
{
    static int fall_counter = 0;
    double tilt_angle = sqrt(state.attitude.com_euler[0] * state.attitude.com_euler[0] + state.attitude.com_euler[1] * state.attitude.com_euler[1]);
    // std::cout<<"tilt_angle" << tilt_angle<<std::endl;
    if (tilt_angle > state.ctrl.tilt_threshold)
    {
        fall_counter++;
    }
    else
    {
        fall_counter = 0;
    }
    if (fall_counter > 400 / MAIN_UPDATE_INTERVEL_MS)
    {
        state.ctrl.safety_halt = true;
        std::cout << COUT_RED << "The robot is going to fall, shutdown program" << COUT_RESET << std::endl;
    }
}

void DogControl::process_walk_mode_cmd(DogState &state, double dt)
{
    double delta_time = MAIN_UPDATE_INTERVEL_MS / 1000.0;
    gait_ptr_->moving_plan = true;
    if(!state.ctrl.doggy_cmd_last.is_walking)
    {
        state.ctrl.com_pos_set = state.attitude.com_pos;
    }
    Eigen::Vector3d new_ang_cmd;
    Eigen::Vector3d new_lin_cmd;
    if (state.ctrl.doggy_cmd.proportional_cmd == true)
    {
        new_ang_cmd = Eigen::Vector3d(0, 0, state.ctrl.doggy_cmd.yaw_rate * gait_ptr_->get_max_vel()(2));
        new_lin_cmd = Eigen::Vector3d(state.ctrl.doggy_cmd.vel_x * gait_ptr_->get_max_vel()(0), state.ctrl.doggy_cmd.vel_y * gait_ptr_->get_max_vel()(1), 0);
    }
    else
    {
        new_ang_cmd = Eigen::Vector3d(0, 0, Utils::clamp(state.ctrl.doggy_cmd.yaw_rate, -gait_ptr_->get_max_vel()(2), gait_ptr_->get_max_vel()(2)));
        new_lin_cmd = Eigen::Vector3d(Utils::clamp(state.ctrl.doggy_cmd.vel_x, -gait_ptr_->get_max_vel()(0), gait_ptr_->get_max_vel()(0)),
                                      Utils::clamp(state.ctrl.doggy_cmd.vel_y, -gait_ptr_->get_max_vel()(1), gait_ptr_->get_max_vel()(1)), 0);
    }

    state.ctrl.com_ang_vel_r_set = state.ctrl.com_ang_vel_r_set + delta_time * Utils::clamp((new_ang_cmd - state.ctrl.com_ang_vel_r_set) / delta_time, -Eigen::Vector3d(state.ctrl.max_ang_acc, state.ctrl.max_ang_acc, state.ctrl.max_ang_acc), Eigen::Vector3d(state.ctrl.max_ang_acc, state.ctrl.max_ang_acc, state.ctrl.max_ang_acc));

    state.ctrl.com_lin_vel_r_set = state.ctrl.com_lin_vel_r_set + delta_time * Utils::clamp((new_lin_cmd - state.ctrl.com_lin_vel_r_set) / delta_time, -Eigen::Vector3d(state.ctrl.max_lin_acc, state.ctrl.max_lin_acc, state.ctrl.max_lin_acc), Eigen::Vector3d(state.ctrl.max_lin_acc, state.ctrl.max_lin_acc, state.ctrl.max_lin_acc));

    if (state.attitude.com_pos(2) > 0.12) // terrain_adaptaion
    {
        state.ctrl.com_euler_set(0) = state.attitude.estimated_terrain_euler(0);
        state.ctrl.com_euler_set(1) = state.attitude.estimated_terrain_euler(1);
    }
    else
    {
        state.ctrl.com_euler_set(0) = 0;
        state.ctrl.com_euler_set(1) = 0;
    } 

    state.ctrl.com_euler_set(2) += state.ctrl.yaw_offset_set;
    state.ctrl.yaw_offset_set = 0;
    state.ctrl.com_ang_vel_set = state.attitude.com_rot_mat * state.ctrl.com_ang_vel_r_set;
    state.ctrl.com_lin_vel_set = state.attitude.com_rot_mat * state.ctrl.com_lin_vel_r_set;
    state.ctrl.com_pos_set += state.attitude.com_rot_mat_z * state.ctrl.com_lin_vel_r_set * dt;
    state.ctrl.com_euler_set(2) += state.ctrl.com_ang_vel_r_set(2) * dt;

    static bool stop_flag = false;
    static Eigen::Vector3d lin_vel_last_cmd = Eigen::Vector3d(0, 0, 0);
    double threshold = 0.1;
    if(new_lin_cmd.lpNorm<2>() < threshold && lin_vel_last_cmd.lpNorm<2>() > threshold)
    {
        stop_flag = true; 
    }
    if(new_lin_cmd.lpNorm<2>() > threshold)
    {
        stop_flag = false;
    }
    if(stop_flag)
    {
        if(state.attitude.com_lin_vel.segment<2>(0).lpNorm<2>() < threshold * 4)
        {
            state.ctrl.com_pos_set = state.attitude.com_pos;
            stop_flag = false;
        }
    }
    lin_vel_last_cmd = new_lin_cmd;

    if (state.ctrl.doggy_cmd.proportional_attitude == true)
    {
        state.ctrl.com_pos_set(2) = gait_ptr_->get_body_height(state) + state.ctrl.doggy_cmd.body_height * controller_ptr_->get_body_height_max_delta();
    }
    else
    {
        state.ctrl.com_pos_set(2) = Utils::clamp(state.ctrl.doggy_cmd.body_height, gait_ptr_->get_body_height(state) - controller_ptr_->get_body_height_max_delta(), gait_ptr_->get_body_height(state) + controller_ptr_->get_body_height_max_delta());
    }

}

void DogControl::process_stance_mode_cmd(DogState &state, double dt)
{
    if(state.ctrl.doggy_cmd_last.is_walking)
    {
        state.ctrl.com_pos_set = state.attitude.com_pos;
    }
    gait_ptr_->moving_plan = false;
    state.ctrl.com_ang_vel_r_set.setZero();
    state.ctrl.com_lin_vel_r_set.setZero();
    state.ctrl.com_ang_vel_set.setZero();
    state.ctrl.com_lin_vel_set.setZero();
    if (state.ctrl.doggy_cmd.proportional_attitude == true)
    {
        state.ctrl.com_euler_set(0) = state.ctrl.doggy_cmd.body_roll * controller_ptr_->get_max_angle()(0);
        state.ctrl.com_euler_set(1) = state.ctrl.doggy_cmd.body_pitch * controller_ptr_->get_max_angle()(1);
        state.ctrl.yaw_offset_set = state.ctrl.doggy_cmd.body_yaw * controller_ptr_->get_max_angle()(2);
        state.ctrl.com_pos_set(2) = gait_ptr_->get_body_height(state) + state.ctrl.doggy_cmd.body_height * controller_ptr_->get_body_height_max_delta();
    }
    else
    {
        state.ctrl.com_euler_set(0) = Utils::clamp(state.ctrl.doggy_cmd.body_roll, -controller_ptr_->get_max_angle()(0), controller_ptr_->get_max_angle()(0));
        state.ctrl.com_euler_set(1) = Utils::clamp(state.ctrl.doggy_cmd.body_pitch, -controller_ptr_->get_max_angle()(1), controller_ptr_->get_max_angle()(1));
        state.ctrl.yaw_offset_set = Utils::clamp(state.ctrl.doggy_cmd.body_yaw, -controller_ptr_->get_max_angle()(2), controller_ptr_->get_max_angle()(2));
        state.ctrl.com_pos_set(2) = Utils::clamp(state.ctrl.doggy_cmd.body_height, gait_ptr_->get_body_height(state) - controller_ptr_->get_body_height_max_delta(), gait_ptr_->get_body_height(state) + controller_ptr_->get_body_height_max_delta());
    }
    if (state.attitude.com_pos(2) > 0.12) // terrain_adaptaion
    {
        state.ctrl.com_euler_set(0) += state.attitude.estimated_terrain_euler(0);
        state.ctrl.com_euler_set(1) += state.attitude.estimated_terrain_euler(1);
    }
}

void DogControl::process_gait_switch_cmd(DogState &state, double dt)
{
    if (!gait_ptr_->ready_to_detach())
    {
        return;
    }
    if (state.ctrl.doggy_cmd.cmd_code == state.ctrl.doggy_cmd.CMD_GAIT_SWITCH_1)
    {
        if (gait_ptr_->get_gait_name() != "walk_gait")
        {
            gait_ptr_ = std::make_unique<WalkGait>();
            std::cout << "switch to walk gait!" << std::endl;
        }
    }
    else if (state.ctrl.doggy_cmd.cmd_code == state.ctrl.doggy_cmd.CMD_GAIT_SWITCH_2)
    {
        if (gait_ptr_->get_gait_name() != "pee")
        {
            gait_ptr_ = std::make_unique<Pee>();
            std::cout << "switch to pee!" << std::endl;
        }
    }
    else if (state.ctrl.doggy_cmd.cmd_code == state.ctrl.doggy_cmd.CMD_GAIT_SWITCH_3)
    {
        if (gait_ptr_->get_gait_name() != "probe_gait")
        {
            gait_ptr_ = anti_slip_.get_probe_gait_ptr(); //std::make_unique<ProbeGait>();
            std::cout << "switch to probe gait!" << std::endl;
        }
    }
}

void DogControl::process_doggy_cmd(DogState &state, double dt)
{
    if (!state.ctrl.doggy_cmd_received)
    {
        return;
    }

    if (state.ctrl.doggy_cmd.is_walking)
    {
        process_walk_mode_cmd(state, dt);
    }
    else
    {
        process_stance_mode_cmd(state, dt);
        process_gait_switch_cmd(state, dt);
    }

    gait_ptr_->gait_cmd_handler(state.ctrl.doggy_cmd.gait_code);

    state.ctrl.com_euler_set(2) = Utils::periodical_clamp(state.ctrl.com_euler_set(2), -PI, PI, 2 * PI);
    state.ctrl.doggy_cmd_last = state.ctrl.doggy_cmd;
}

void DogControl::anti_slip(DogState &state)
{
    anti_slip_.slip_detect(state);
    anti_slip_.friction_probe(state);
}

void DogControl::write_log(DogState &state)
{
    robot_log_.write_log(state);
}

