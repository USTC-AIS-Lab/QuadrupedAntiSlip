#include "pee.h"

// FL, FR, RL, RR
/*
    0 1
    2 3
*/
//|**stance**|**swing**|

Pee::Pee()
{
    gait_name_ = "pee";
    ros::NodeHandle nh;
    nh.getParam(gait_name_ + "/max_vel_x", max_vel(0));
    nh.getParam(gait_name_ + "/max_vel_y", max_vel(1));
    nh.getParam(gait_name_ + "/max_vel_yaw_rate", max_vel(2));
    gait_spansteps_ << 800, 800, 800, 800;
    pee_leg = 3;
    swing_spansteps_ << 0, 0, 0, 0;
    initial_counter_<< 0, 0, 0, 0;
    gait_counter_ = initial_counter_;
    swing_spansteps_(pee_leg) = gait_spansteps_(pee_leg) * 0.8;
    stance_spansteps_ = gait_spansteps_ - swing_spansteps_;
    start = false;
    peeing = false;
    finishing = false;
    pee_time = stance_spansteps_(pee_leg) + swing_spansteps_(pee_leg) * 0.5;
    pee_foot_offset_abs << 0.0, 0.14 * (1 - 2 * (pee_leg % 2)), -0.08;
    com_offset_ratio = 1.5;
    gait_counter_reset();
}

void Pee::counter_contact_plan(DogState &state)
{
    if (start)
    {
        for (int i = 0; i < NUM_LEG; ++i)
        {
            is_moving[i] = true;
        }
    }

    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (i != pee_leg)
        {
            state.ctrl.plan_contact[i] = true;
            is_moving[i] = false;
        }
        else
        {
            if (!is_moving[i])
            {
                state.ctrl.plan_contact[i] = true;
            }
            else
            {
                if(!peeing)
                {
                    if(gait_counter_(i) == pee_time)
                    {
                        peeing = true;
                        std::cout << "start peeing!" << std::endl; 
                    }
                    gait_counter_(i) += 1; // holding position
                }
                if(finishing)
                {
                    peeing = false;
                }
                gait_counter_(i) = std::fmod(gait_counter_(i), gait_spansteps_(i));
                state.ctrl.plan_contact[i] = gait_counter_(i) <= stance_spansteps_(i);
                if (finishing && state.ctrl.plan_contact[i])
                {
                    is_moving[i] = false;
                    gait_counter_(i) = initial_counter_(i);
                    finishing = false;
                    start = false;
                }
            }
        }
    }
}

void Pee::compute_foot_kinematics(DogState &state)
{
    Eigen::Matrix<double, 1, NUM_LEG> curve_time = Eigen::Matrix<double, 1, NUM_LEG>::Zero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (i != pee_leg)
        {
            state.ctrl.foot_pos_abs_set.block<3, 1>(0, i) = state.attitude.foot_pos_abs.block<3, 1>(0, i);
            state.ctrl.foot_vel_r_set.block<3, 1>(0, i).setZero();
        }
        else
        {
            if (gait_counter_(i) <= stance_spansteps_(i)) // ground contact phase
            {
                curve_time(i) = 0.0;
                foot_pos_start_r.block<3, 1>(0, i) = state.attitude.foot_pos_r.block<3, 1>(0, i);
                foot_pos_start_abs.block<3, 1>(0, i) = state.attitude.com_rot_mat_r_to_abs * foot_pos_start_r.block<3, 1>(0, i);
                // keep updating start pos, after the leg switches to swing phase, foot_pos_start_abs will be the last updated value.
            }
            else
            {
                if(!finishing)
                {
                    curve_time(i) = std::min((double)(gait_counter_(i) - stance_spansteps_(i)) / (double)(pee_time - stance_spansteps_(i)), 1.0);
                }
                else
                {
                    curve_time(i) = std::min(1.3 * (double)(gait_counter_(i) - pee_time) / (double)(gait_spansteps_(i) - pee_time), 1.0);
                }
            }
            std::pair<Eigen::Vector3d, Eigen::Vector3d> pos_vel_abs = bezierUtil.get_foot_pos_curve(curve_time(i),
                                                                                                    foot_pos_start_abs.block<3, 1>(0, i),
                                                                                                    foot_pos_target_abs.block<3, 1>(0, i), 0.0, 0, 0);
            state.ctrl.foot_pos_abs_set.block<3, 1>(0, i) = pos_vel_abs.first;
            if(peeing)
            {
                foot_pos_start_r.block<3, 1>(0, i) = state.attitude.foot_pos_r.block<3, 1>(0, i);
                foot_pos_start_abs.block<3, 1>(0, i) = state.attitude.com_rot_mat_r_to_abs * foot_pos_start_r.block<3, 1>(0, i);
                state.ctrl.foot_vel_r_set.block<3, 1>(0, i) = - state.attitude.com_rot_mat.transpose() * (state.attitude.com_lin_vel_filtered + Utils::skew(state.attitude.com_ang_vel) * (state.attitude.foot_pos.block<3, 1>(0, i) - state.attitude.com_pos)); 
                
            }
            else
            {
                state.ctrl.foot_vel_r_set.block<3, 1>(0, i) = state.attitude.com_rot_mat_r_to_abs.transpose() * (pos_vel_abs.second / (swing_spansteps_(i) * MAIN_UPDATE_INTERVEL_MS / 1000.0))- state.attitude.com_rot_mat.transpose() * (state.attitude.com_lin_vel_filtered + Utils::skew(state.attitude.com_ang_vel) * (state.attitude.foot_pos.block<3, 1>(0, i) - state.attitude.com_pos)); 
            }
        }
    }
    // std::cout << "state.ctrl.foot_forces_kin_set.block<3, 1>(0, pee_leg).transpose()" << std::endl;
    // std::cout << state.ctrl.foot_forces_kin_set.block<3, 1>(0, pee_leg).transpose() << std::endl;
}

void Pee::compute_swing_legs_kinematics(DogState &state, double dt)
{
    counter_contact_plan(state);
    foot_pos_target_abs = state.attitude.foot_pos_abs;
    if(!finishing)
    {
        foot_pos_target_abs.block<3, 1>(0, pee_leg) = state.attitude.com_rot_mat_r_to_abs * state.attitude.hip_pos_r.block<3, 1>(0, pee_leg) + pee_foot_offset_abs;
    }
    else
    {
        foot_pos_target_abs.block<3, 1>(0, pee_leg) = state.attitude.hip_pos_abs.block<3, 1>(0, pee_leg) + Eigen::Vector3d(-0.0, -0.0, -state.attitude.hip_pos(2));
    }
    
    compute_foot_kinematics(state);
    state.ctrl.foot_target_set = state.attitude.com_rot_mat * state.attitude.com_rot_mat_r_to_abs.transpose() * foot_pos_target_abs + state.attitude.com_pos_four_cols;
    
    state.ctrl.foot_pos_r_set = state.attitude.com_rot_mat_r_to_abs.transpose() * state.ctrl.foot_pos_abs_set;
    state.ctrl.foot_pos_set = state.attitude.com_rot_mat * state.ctrl.foot_pos_r_set + state.attitude.com_pos_four_cols;
}

void Pee::gait_counter_reset()
{
    gait_counter_ = initial_counter_;
}

double Pee::get_contact_plan(double dt, double window, int leg_index)
{
    if (!is_moving[leg_index])
    {
        return 1.0;
    }
    else
    {
        return predict_gait_counter(dt, leg_index, true) <= stance_spansteps_(leg_index);
    }
}

Eigen::Vector4d Pee::calculate_foot_support_weight()
{
    Eigen::Vector4d weights;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (!is_moving[i])
        {
            weights[i] = 1.0;
        }
        else
        {
            //int deviation = std::min(abs(gait_counter_(i) - contact_middle), abs(gait_counter_(i) + gait_spansteps_(i) - contact_middle));
            if(gait_counter_(i) < pee_time)
            {
                weights(i) = std::max((stance_spansteps_(i) - gait_counter_(i)) / (double)stance_spansteps_(i), 0.0);
            }
            else
            {
                weights(i) = std::max((gait_counter_(i) - gait_spansteps_(i) * 0.8) / (gait_spansteps_(i) * 0.2), 0.0);
            }
            
        }
        weights(i) = std::erf(weights(i) * 2);
    }

    // std::cout << "stance_span_weights.transpose()" << std::endl;
    // std::cout << stance_span_weights.transpose() << std::endl;
    // weights(pee_leg) = 0.0;

    return weights;
}

std::vector<Eigen::Matrix<double, 3, NUM_LEG>> Pee::predict_foot_poses(Eigen::VectorXd reference_trajectory, int state_dim, DogState &state, double dt)
{
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> empty_value;
    return empty_value;
}

void Pee::gait_cmd_handler(int cmd)
{
    if(cmd == doggy_msgs::DoggyMove::GAIT_MOVE_1)
    {
        finishing = true; // stop peeing, retract leg.
    }
    else if(cmd == doggy_msgs::DoggyMove::GAIT_MOVE_2)
    {
        start = true; // start peeing
    }
}