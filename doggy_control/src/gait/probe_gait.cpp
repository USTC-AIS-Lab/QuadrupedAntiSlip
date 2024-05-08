#include "probe_gait.h"

// FL, FR, RL, RR
/*
    0 1
    2 3
*/
//|stance****************|*swing******[probe_tick]*********|

ProbeGait::ProbeGait()
{
    gait_name_ = "probe_gait";
    ros::NodeHandle nh;
    double gait_spansteps;
    probe_foot_offset_abs.setZero();
    nh.getParam(gait_name_ + "/probe_leg", probe_leg_);
    nh.getParam(gait_name_ + "/gait_spansteps", gait_spansteps);
    nh.getParam(gait_name_ + "/com_offset_ratio", com_offset_ratio);
    nh.getParam(gait_name_ + "/probe_foot_offset_abs_x", probe_foot_offset_abs(0));
    nh.getParam(gait_name_ + "/probe_foot_offset_abs_y", probe_foot_offset_abs(1));
    nh.getParam(gait_name_ + "/foot_height", foot_height);
    nh.getParam(gait_name_ + "/body_height", body_height_);
    
    is_probing = false;
    start_probe = false;
    finishing_probe = false;
    gait_spansteps_ << gait_spansteps, gait_spansteps, gait_spansteps, gait_spansteps;
    swing_spansteps_ << 0, 0, 0, 0;
    initial_counter_<< 0, 0, 0, 0;
    gait_counter_ = initial_counter_;
    swing_spansteps_(probe_leg_) = gait_spansteps_(probe_leg_) * 0.8;
    stance_spansteps_ = gait_spansteps_ - swing_spansteps_;
    probe_tick = stance_spansteps_(probe_leg_) + swing_spansteps_(probe_leg_) * 0.5;
    com_offset_ratio = 2.5;
    foot_height = 0.1;
    gait_counter_reset();
}

void ProbeGait::counter_contact_plan(DogState &state)
{
    
    for (int i = 0; i < NUM_LEG; ++i)
    {
        is_moving[i] = (i == probe_leg_ && start_probe);

        if (i != probe_leg_)
        {
            state.ctrl.plan_contact[i] = true;
        }
        else
        {
            if (!is_moving[i])
            {
                state.ctrl.plan_contact[i] = true;
            }
            else
            {
                if(!is_probing && start_probe)
                {
                    if(gait_counter_(i) == probe_tick)
                    {
                        is_probing = true;
                        std::cout << "reach probe position." << std::endl; 
                    }
                    gait_counter_(i) += 1; 
                }
                if(finishing_probe)
                {
                    is_probing = false;
                }
                gait_counter_(i) = std::fmod(gait_counter_(i), gait_spansteps_(i));
                state.ctrl.plan_contact[i] = gait_counter_(i) <= stance_spansteps_(i);
                if(i == probe_leg_ && is_probing)
                {
                    state.ctrl.plan_contact[i] = true;
                }
                if (finishing_probe && state.ctrl.plan_contact[i])
                {
                    is_moving[i] = false;
                    gait_counter_(i) = initial_counter_(i);
                    finishing_probe = false;
                    start_probe = false;
                    std::cout << "probe leg retracted." << std::endl;
                }
            }
        }
    }
    // std::cout << "state.contact(probe_leg_): " << state.contact(probe_leg_) << std::endl;
    // std::cout << gait_counter_(probe_leg_) << ", " << state.ctrl.plan_contact[probe_leg_] << std::endl;
    // std::cout << "state.attitude.early_contact[probe_leg_]: " << state.attitude.early_contact[probe_leg_] << std::endl;
}

double ProbeGait::get_contact_plan(double dt, double window, int leg_index)
{
    if(leg_index == probe_leg_ && is_probing)
    {
        return true;
    }
    else
    {
        return !is_moving[leg_index];
    }
    
}


void ProbeGait::compute_foot_kinematics(DogState &state)
{
    Eigen::Matrix<double, 1, NUM_LEG> curve_time = Eigen::Matrix<double, 1, NUM_LEG>::Zero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (i != probe_leg_)
        {
            state.ctrl.foot_pos_set.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i);
            state.ctrl.foot_vel_r_set.block<3, 1>(0, i).setZero();
        }
        else // for probe leg
        {
            if (gait_counter_(i) <= stance_spansteps_(i)) // ground contact phase
            {
                curve_time(i) = 0.0;
                foot_pos_start.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i);
                // keep updating start pos, after the leg switches to swing phase, foot_pos_start_abs will be the last updated value.
            }
            else
            {
                if(!finishing_probe) // first half
                {
                    curve_time(i) = std::min((double)(gait_counter_(i) - stance_spansteps_(i)) / (double)(probe_tick - stance_spansteps_(i)), 1.0);
                }
                else
                {
                    curve_time(i) = std::min((double)(gait_counter_(i) - probe_tick) / (double)(gait_spansteps_(i) - probe_tick), 1.0);
                }// use 1.3 here to speed up leg retraction.
            }
            std::pair<Eigen::Vector3d, Eigen::Vector3d> pos_vel = bezierUtil.get_foot_pos_curve(curve_time(i),
                                                                                                    foot_pos_start.block<3, 1>(0, i),
                                                                                                    foot_pos_target.block<3, 1>(0, i), 
                                                                                                    foot_height * 0.3, foot_height, 0);
            state.ctrl.foot_pos_set.block<3, 1>(0, i) = pos_vel.first;
            // state.ctrl.foot_pos_set.block<3, 1>(0, i)(2) = state.attitude.foot_pos.block<3, 1>(0, i)(2);
            if(is_probing)
            {
                foot_pos_start.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i); // keep updating start pos for later leg retraction.
                state.ctrl.foot_vel_r_set.block<3, 1>(0, i) = - state.attitude.com_rot_mat.transpose() * (state.attitude.com_lin_vel_filtered + Utils::skew(state.attitude.com_ang_vel) * (state.attitude.foot_pos.block<3, 1>(0, i) - state.attitude.com_pos)); 
            }
            else
            {
                int flight_span;
                if(!finishing_probe)
                {
                    flight_span = probe_tick - stance_spansteps_(i);
                }
                else
                {
                    flight_span = gait_spansteps_(i) - probe_tick;
                }
                state.ctrl.foot_vel_r_set.block<3, 1>(0, i) = state.attitude.com_rot_mat.transpose() * (pos_vel.second / (flight_span * MAIN_UPDATE_INTERVEL_MS / 1000.0)) - state.attitude.com_rot_mat.transpose() * (state.attitude.com_lin_vel_filtered + Utils::skew(state.attitude.com_ang_vel) * (state.attitude.foot_pos.block<3, 1>(0, i) - state.attitude.com_pos)); 
            }
        }
    }
}

void ProbeGait::compute_swing_legs_kinematics(DogState &state, double dt)
{
    counter_contact_plan(state);
    for (int i = 0; i < NUM_LEG; ++i)
    {
        
        if (i == probe_leg_ && start_probe)
        {
            if (gait_counter_(i) == stance_spansteps_(i)) // only set the target once. the target remain constant during probe.
            {
                foot_pos_target.block<3, 1>(0, i) = state.attitude.hip_pos.block<3, 1>(0, i) + state.attitude.com_rot_mat * state.attitude.com_rot_mat_r_to_abs.transpose() * probe_foot_offset_abs;
                foot_pos_target.block<3, 1>(0, i)(2) = 0;
            }
            else if(finishing_probe)
            {
                foot_pos_target.block<3, 1>(0, i) = state.attitude.hip_pos.block<3, 1>(0, i);
                foot_pos_target.block<3, 1>(0, i)(2) = 0;
            }
        }
        else
        {
            foot_pos_target.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i);
        }
    }
    

    compute_foot_kinematics(state);
    state.ctrl.foot_target_set = foot_pos_target;
    state.ctrl.foot_pos_r_set = state.attitude.com_rot_mat.transpose() * (state.ctrl.foot_pos_set - state.attitude.com_pos_four_cols);

    if(state.slip_info.probe_slid_detected)
    {
        state.slip_info.probe_slid_detected = false;
        finishing_probe = true;
        is_probing = false;
    }

    state.slip_info.is_probing = is_probing;
    state.slip_info.probe_leg = probe_leg_;
    

}

void ProbeGait::gait_counter_reset()
{
    gait_counter_ = initial_counter_;
}

Eigen::Vector4d ProbeGait::calculate_foot_support_weight()
{
    Eigen::Vector4d weights;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (i == probe_leg_)
        {
            weights(i) = 0.0;
        }
        else
        {
            weights(i) = 1.0;
            
        }
    }

    return weights;
}


void ProbeGait::gait_cmd_handler(int cmd)
{
    if(cmd == doggy_msgs::DoggyMove::GAIT_MOVE_2)
    {
        if(!finishing_probe)
        {
            finishing_probe = true; 
            std::cout << "finish probe." << std::endl;
        }
        
    }
    else if(cmd == doggy_msgs::DoggyMove::GAIT_MOVE_1)
    {
        if(!start_probe)
        {
            start_probe = true; 
            std::cout << "start probe." << std::endl;
        }
    }
}

std::vector<Eigen::Matrix<double, 3, NUM_LEG>> ProbeGait::predict_foot_poses(Eigen::VectorXd reference_trajectory, int state_dim, DogState &state, double dt)
{
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> empty_value;
    return empty_value; // foot pose prediction is not needed for this gait
}

bool ProbeGait::ready_to_engage()
{
    return true;
}

bool ProbeGait::ready_to_detach()
{
    return !start_probe;
}

