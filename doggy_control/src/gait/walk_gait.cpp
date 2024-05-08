#include "walk_gait.h"
#include <unsupported/Eigen/SpecialFunctions>

// FL, FR, RL, RR
/*
    0 1
    2 3
*/
//|**swing**|**stance**|
WalkGait::WalkGait()
{
    gait_name_ = "walk_gait";
    foot_delta_x_limit = 0.25;
    foot_delta_y_limit = 0.25;
    ros::NodeHandle nh;
    nh.getParam(gait_name_ + "/max_vel_x", max_vel(0));
    nh.getParam(gait_name_ + "/max_vel_y", max_vel(1));
    nh.getParam(gait_name_ + "/max_vel_yaw_rate", max_vel(2));
    nh.getParam(gait_name_ + "/foot_height", foot_height);
    nh.getParam(gait_name_ + "/touch_down", touch_down);
    nh.getParam(gait_name_ + "/gait_span_fl", gait_spansteps_(0));
    nh.getParam(gait_name_ + "/gait_span_fr", gait_spansteps_(1));
    nh.getParam(gait_name_ + "/gait_span_rl", gait_spansteps_(2));
    nh.getParam(gait_name_ + "/gait_span_rr", gait_spansteps_(3));
    nh.getParam(gait_name_ + "/initial_phase_fl", initial_phase(0));
    nh.getParam(gait_name_ + "/initial_phase_fr", initial_phase(1));
    nh.getParam(gait_name_ + "/initial_phase_rl", initial_phase(2));
    nh.getParam(gait_name_ + "/initial_phase_rr", initial_phase(3));
    nh.getParam(gait_name_ + "/swing_phase_fl", swing_phase(0));
    nh.getParam(gait_name_ + "/swing_phase_fr", swing_phase(1));
    nh.getParam(gait_name_ + "/swing_phase_rl", swing_phase(2));
    nh.getParam(gait_name_ + "/swing_phase_rr", swing_phase(3));
    nh.getParam(gait_name_ + "/com_offset_ratio", com_offset_ratio); 
    
    for (int i = 0; i < NUM_LEG; i++)
    {
        swing_spansteps_(i) = gait_spansteps_(i) * swing_phase(i);
        initial_counter_(i) = gait_spansteps_(i) * initial_phase(i);
        pregait_counter_(i) = -200;
    }
    stance_spansteps_ = gait_spansteps_ - swing_spansteps_;
    terrain_angle.setZero();
    gait_counter_reset();
}


void WalkGait::gait_counter_reset()
{
    
    for (int i = 0; i < NUM_LEG; i++)
    {
        swing_spansteps_(i) = gait_spansteps_(i) * swing_phase(i);
        initial_counter_(i) = gait_spansteps_(i) * initial_phase(i);
    }
    for (int i = 0; i < NUM_LEG; i++)
    {
        foot_start_allocated[i] = false;
        is_moving[i] = false;
    }
    gait_counter_ = pregait_counter_;
}

Eigen::Matrix<double, 3, NUM_LEG> WalkGait::predict_foot_pose(Eigen::Matrix<double, 3, NUM_LEG> hip_pos, Eigen::Vector3d com_lin_vel, Eigen::Vector3d com_lin_vel_set, Eigen::Vector3d com_ang_vel, Eigen::Vector3d com_ang_vel_set, Eigen::Vector4i counter_now)
{
    Eigen::Matrix<double, 3, NUM_LEG> predicted_foot_pose;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pose_target = calculate_foot_pos_target(hip_pos, terrain_angle, com_lin_vel, com_lin_vel_set, com_ang_vel, com_ang_vel_set, counter_now);
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if(is_contact_phase(counter_now(i), i)) // contact phase
        {
            predicted_foot_pose.block<3, 1>(0, i) = foot_pose_target.block<3, 1>(0, i);
        }
        else // swing phase
        {

            Eigen::Matrix<double, 3, NUM_LEG> foot_pose_target_last = calculate_foot_pos_target(hip_pos, terrain_angle, -com_lin_vel, -com_lin_vel_set, -com_ang_vel, -com_ang_vel_set, swing_spansteps_ - counter_now);
            double curve_time = (double)counter_now(i) / (double)swing_spansteps_(i); // get swing time pos (0.0 ~ 1.0)
            std::pair<Eigen::Vector3d, Eigen::Vector3d> pos_vel = bezierUtil.get_foot_pos_curve(curve_time,
                                                                                                foot_pose_target_last.block<3, 1>(0, i),
                                                                                                foot_pose_target.block<3, 1>(0, i), 
                                                                                                foot_height * 0.3, foot_height, 0);
            predicted_foot_pose.block<3, 1>(0, i) = pos_vel.first;

            // in order to make this fast
            // predicted_foot_pose.block<3, 1>(0, i) = foot_pose_target.block<3, 1>(0, i) + Eigen::Vector3d(0, 0, 0.1);
            
        }
    }
    return predicted_foot_pose;
}

Eigen::Matrix<double, 3, NUM_LEG> WalkGait::calculate_foot_pos_target(Eigen::Matrix<double, 3, NUM_LEG> hip_pos, Eigen::Vector3d terrain_angle, Eigen::Vector3d com_lin_vel, Eigen::Vector3d com_lin_vel_set, Eigen::Vector3d com_ang_vel, Eigen::Vector3d com_ang_vel_set, Eigen::Vector4i counter_now)
{

    Eigen::Matrix<double, 3, NUM_LEG> pos_target = hip_pos;
    Eigen::Vector3d pose = (hip_pos.block<3, 1>(0, 0) + hip_pos.block<3, 1>(0, 1) + hip_pos.block<3, 1>(0, 2) + hip_pos.block<3, 1>(0, 3)) / 4.0; // the pose can be computed by the hip pose
    Eigen::Matrix<double, 3, NUM_LEG> hip_offset; // hip offset from com
    for (int i = 0; i < NUM_LEG; ++i)
    {
        hip_offset.block<3, 1>(0, i) = hip_pos.block<3, 1>(0, i) - pose;
    }
    
    for (int i = 0; i < NUM_LEG; ++i)
    {
        
        Eigen::Vector3d pos_target_delta = Utils::raibert_heuristic_delta_pos(hip_offset.block<3, 1>(0, i), com_lin_vel, com_lin_vel_set, com_ang_vel, com_ang_vel_set, (stance_spansteps_(i) * MAIN_UPDATE_INTERVEL_MS / 1000.0), foot_delta_x_limit, foot_delta_y_limit);
        // std::cout << pos_target_delta.transpose()<< std::endl;
        if(is_contact_phase(counter_now(i), i))// contact phase
        {
            pos_target.block<3, 1>(0, i) = hip_pos.block<3, 1>(0, i) + pos_target_delta * (1.0 - 2.0 * (((double)(counter_now(i) - swing_spansteps_(i))) / (double)(stance_spansteps_(i))));

            // pos_target.block<3, 1>(0, i) = foot_pos_start.block<3, 1>(0, i);
            
        }
        else // swing phase
        {
            double contact_dt = (swing_spansteps_(i) - counter_now(i)) * MAIN_UPDATE_INTERVEL_MS / 1000.0; // predicted time to contact
            Eigen::Vector3d predicted_offset;
            predicted_offset.setZero();
            predicted_offset.segment<2>(0) = Utils::calculate_displacement(Eigen::Vector2d(com_lin_vel(0) + (Utils::skew(com_ang_vel) * hip_offset.block<3, 1>(0, i))(0), com_lin_vel(1) + (Utils::skew(com_ang_vel) * hip_offset.block<3, 1>(0, i))(1)), com_ang_vel(2), contact_dt);
            // predicted_offset = Utils::euler_to_quat(0.5 * contact_dt * Eigen::Vector3d(0, 0, com_ang_vel_r(2))) * (contact_dt * com_lin_vel_r);
            pos_target.block<3, 1>(0, i) = hip_pos.block<3, 1>(0, i) + Utils::euler_to_quat(contact_dt * Eigen::Vector3d(0, 0, com_ang_vel(2))).toRotationMatrix() * pos_target_delta + predicted_offset;
            
        }
        pos_target(2, i) = 0;
    }
    Eigen::Matrix<double, 3, NUM_LEG> terrain_adapt_offset;
    terrain_adapt_offset.setZero();
    terrain_adapt_offset(2, 0) = + abs(hip_offset(1, 0)) * std::tan(terrain_angle(0)) - abs(hip_offset(0, 0)) * std::tan(terrain_angle(1));
    terrain_adapt_offset(2, 1) = - abs(hip_offset(1, 1)) * std::tan(terrain_angle(0)) - abs(hip_offset(0, 1)) * std::tan(terrain_angle(1));
    terrain_adapt_offset(2, 2) = + abs(hip_offset(1, 2)) * std::tan(terrain_angle(0)) + abs(hip_offset(0, 2)) * std::tan(terrain_angle(1));
    terrain_adapt_offset(2, 3) = - abs(hip_offset(1, 3)) * std::tan(terrain_angle(0)) + abs(hip_offset(0, 3)) * std::tan(terrain_angle(1));
    pos_target += terrain_adapt_offset;
    return pos_target;
}

bool WalkGait::is_contact_phase(int counter_now, int foot_num)
{
    if(counter_now < 0)
    {
        return true;
    }
    else
    {
        return std::fmod(counter_now, gait_spansteps_(foot_num)) >= swing_spansteps_(foot_num);
    }
}

void WalkGait::counter_contact_plan(DogState &state)
{
    if (moving_plan)
    {
        for (int i = 0; i < NUM_LEG; ++i)
        {
            is_moving[i] = true;
        }
    }
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (!is_moving[i])
        {
            state.ctrl.plan_contact[i] = true;
            foot_start_allocated[i] = false;
            //gait_counter_reset();
        }
        else
        {
            gait_counter_(i) += 1;
            if(gait_counter_(i) > 0)
            {
                gait_counter_(i) = std::fmod(gait_counter_(i), gait_spansteps_(i));
            }
            if(gait_counter_(i) == -1)
            {
                gait_counter_(i) = initial_counter_(i);
            }
            
            state.ctrl.plan_contact[i] = is_contact_phase(gait_counter_(i), i);
            if (!moving_plan && state.ctrl.plan_contact[i])
            {
                is_moving[i] = false;
                gait_counter_(i) = pregait_counter_(i);

            }
        }
        state.attitude.early_contact[i] = !state.ctrl.plan_contact[i] && state.attitude.is_contact[i] && gait_counter_(i) >= 0.5 * swing_spansteps_(i);
    }
}

void WalkGait::compute_foot_kinematics(DogState &state)
{
    Eigen::Matrix<double, 1, NUM_LEG> curve_time = Eigen::Matrix<double, 1, NUM_LEG>::Zero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (is_contact_phase(gait_counter_(i), i) || !is_moving[i]) // ground contact phase
        {
            if(gait_counter_(i) == swing_spansteps_(i) || !is_moving[i])
            {
                foot_pos_first_contact.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i);
                foot_pos_first_contact.block<3, 1>(0, i)(2) = 0;
            }
            curve_time(i) = 0.0;
            foot_pos_start.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i);
            // foot_pos_start_abs.block<3, 1>(0, i) = state.attitude.com_rot_mat_r_to_abs * foot_pos_start_r.block<3, 1>(0, i);
            // keep updating start pos, after the leg switches to swing phase, foot_pos_start_abs will be the last updated value.
            // foot_start_allocated[i] = true; // make sure the robot uses its initial leg pos as start pos when it start moving_plan.
        }
        if (! is_contact_phase(gait_counter_(i), i) ) // swing phase
        {
            curve_time(i) = std::max(0.0, std::min((double)(gait_counter_(i)) / (double)(swing_spansteps_(i)), 1.0)); // get swing time pos (0.0 ~ 1.0)
        }
        std::pair<Eigen::Vector3d, Eigen::Vector3d> pos_vel = bezierUtil.get_foot_pos_curve(curve_time(i),
                                                                                            foot_pos_start.block<3, 1>(0, i),
                                                                                            foot_pos_target.block<3, 1>(0, i),
                                                                                            foot_height * 0.3, foot_height, 0.0);
        if(state.ctrl.plan_contact[i])
        {
            state.ctrl.foot_pos_set.block<3, 1>(0, i) = foot_pos_first_contact.block<3, 1>(0, i);
            state.ctrl.foot_vel_r_set.block<3, 1>(0, i).setZero();
        }
        else
        {
            state.ctrl.foot_pos_set.block<3, 1>(0, i) = pos_vel.first;
            state.ctrl.foot_vel_r_set.block<3, 1>(0, i) = state.attitude.com_rot_mat.transpose() * (pos_vel.second / (swing_spansteps_(i) * MAIN_UPDATE_INTERVEL_MS / 1000.0))- state.attitude.com_rot_mat.transpose() * (state.attitude.com_lin_vel_filtered + Utils::skew(state.attitude.com_ang_vel) * (state.attitude.foot_pos.block<3, 1>(0, i) - state.attitude.com_pos)); 
        }
        
    }
}

void WalkGait::compute_swing_legs_kinematics(DogState &state, double dt)
{
    terrain_angle = state.attitude.estimated_terrain_euler;
    counter_contact_plan(state);
    foot_pos_target = calculate_foot_pos_target(state.attitude.hip_pos,terrain_angle ,state.attitude.com_lin_vel_filtered, state.ctrl.com_lin_vel_set, state.attitude.com_ang_vel_filtered, state.ctrl.com_ang_vel_set, gait_counter_);
    for(int i = 0 ; i < NUM_LEG; ++i)
    {
        foot_pos_target(2, i) -= touch_down;
    }
    compute_foot_kinematics(state);
    for(int i = 0 ; i < NUM_LEG; ++i)
    {
        if(state.ctrl.plan_contact[i])
        {
            state.ctrl.foot_target_set.block<3, 1>(0, i) = foot_pos_first_contact.block<3, 1>(0, i);
            state.ctrl.foot_target_set(2, i) -= touch_down;
        }
        else
        {  
            state.ctrl.foot_target_set.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);
        }
        
    }
    state.ctrl.foot_pos_r_set = state.attitude.com_rot_mat.transpose() * (state.ctrl.foot_pos_set - state.attitude.com_pos_four_cols);
}


double WalkGait::get_contact_plan(double dt, double window, int leg_index)
{
    if (!is_moving[leg_index])
    {
        return 1.0;
    }
    else
    {
        int window_step = window * 1000.0 / MAIN_UPDATE_INTERVEL_MS;
        int predict_counter_moded_now = predict_gait_counter(dt, leg_index, true);
        int predict_counter_moded_next = predict_gait_counter(dt + window, leg_index, true);
        if((predict_counter_moded_now - swing_spansteps_(leg_index)) * (predict_counter_moded_next - swing_spansteps_(leg_index)) >= 0) // in a same phase
        {
            return predict_counter_moded_now >= swing_spansteps_(leg_index);
        }
        else
        {
            if(predict_counter_moded_now < swing_spansteps_(leg_index)) // entering contact
            {
                return ((double)(predict_counter_moded_next - swing_spansteps_(leg_index))) / (double)window_step;
            }
            else // leaving contact
            {
                return ((double)(gait_spansteps_(leg_index) - predict_counter_moded_now)) / ((double)window_step * 1.4);
            }
        }
        
        
    }
}

std::vector<Eigen::Matrix<double, 3, NUM_LEG>> WalkGait::predict_foot_poses(Eigen::VectorXd reference_trajectory, int state_dim, DogState &state, double dt)
{
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> predicted_foot_poses;
    int T = reference_trajectory.rows() / state_dim;
    for (int time_step = 0; time_step < T; ++time_step)
    {
        Eigen::Vector4i predicted_gait_counter;
        Eigen::Vector4i predicted_gait_counter_unmoded;
        for (int i = 0; i < NUM_LEG; ++i)
        {
            predicted_gait_counter(i) = predict_gait_counter(dt * time_step, i, true);
            predicted_gait_counter_unmoded(i) = predict_gait_counter(dt * time_step, i, false);
        }
        // std::cout << time_step << ": " << predicted_gait_counter.transpose() << std::endl;
        Eigen::Vector3d pose = reference_trajectory.segment<3>(state_dim * time_step + 3);
        Eigen::Matrix<double, 3, NUM_LEG> pos_four_cols;
        for(int i = 0; i < NUM_LEG; ++i)
        {
            pos_four_cols.block<3, 1>(0, i) = pose;
        }
        Eigen::Vector3d euler_now = reference_trajectory.segment<3>(state_dim * time_step + 0);
        Eigen::Vector3d com_lin_vel;
        Eigen::Vector3d com_ang_vel;
        if (time_step < T - 1)
        {
            com_lin_vel = (reference_trajectory.segment<3>(state_dim * (time_step + 1) + 3) - reference_trajectory.segment<3>(state_dim * time_step + 3)) / dt;
            Eigen::Vector3d euler_next = reference_trajectory.segment<3>(state_dim * (time_step + 1) + 0);
            com_ang_vel = Utils::euler_to_angular_velocity_mapping(euler_now) * (euler_next - euler_now) / dt;
        }
        else
        {
            com_lin_vel = (reference_trajectory.segment<3>(state_dim * (time_step) + 3) - reference_trajectory.segment<3>(state_dim * (time_step - 1) + 3)) / dt;
            Eigen::Vector3d euler_last = reference_trajectory.segment<3>(state_dim * (time_step - 1) + 0);
            com_ang_vel = Utils::euler_to_angular_velocity_mapping(euler_now) * (euler_now - euler_last) / dt;
        }

        Eigen::Matrix<double, 3, NUM_LEG> hip_pos = Utils::euler_to_quat(euler_now).toRotationMatrix() * state.attitude.hip_pos_r + pos_four_cols;

        Eigen::Matrix<double, 3, NUM_LEG> foot_pose = predict_foot_pose(hip_pos, com_lin_vel, com_lin_vel, com_ang_vel, com_ang_vel, predicted_gait_counter);
        for (int i = 0; i < NUM_LEG; ++i)
        {
            // std::cout << i << ": " << abs(predicted_gait_counter_unmoded(i) - gait_counter_(i)) << std::endl;
            if (abs(predicted_gait_counter_unmoded(i) - gait_counter_(i)) < gait_spansteps_(i) * 0.05 || !is_moving[i])
            {
                foot_pose.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i);
            }
        }
        predicted_foot_poses.emplace_back(foot_pose);
    }
    return predicted_foot_poses;
}

//|**swing**|**stance**|

Eigen::Vector4d WalkGait::calculate_foot_support_weight()
{
    Eigen::Vector4d weights;
    Eigen::Vector4d stance_span_weights;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (!is_moving[i])
        {
            weights[i] = 1.0;
            stance_span_weights[i] = 1.0;
        }
        else
        {
            int gait_counter_now = gait_counter_(i);
            if(gait_counter_now < 0)
            {
                gait_counter_now = (gait_counter_now + initial_counter_(i)) % gait_spansteps_(i) ;
            }
            int contact_middle = swing_spansteps_(i) + stance_spansteps_(i) * 0.5;
            int deviation = std::min(abs(gait_counter_now - contact_middle), abs(gait_counter_now + gait_spansteps_(i) - contact_middle));
            weights(i) = (gait_spansteps_(i) * 0.5 - deviation) / (gait_spansteps_(i) * 0.5);
            stance_span_weights(i) = stance_spansteps_.cast<double>()(i) / stance_spansteps_.maxCoeff();
        }
        weights(i) = std::erf(weights(i) * 2);
    }

    // std::cout << "stance_span_weights.transpose()" << std::endl;
    // std::cout << stance_span_weights.transpose() << std::endl;
    weights = weights.cwiseProduct(stance_span_weights);
    // weights(0) = 0.0;

    return weights;
}

void WalkGait::gait_cmd_handler(int cmd)
{
    if(cmd == doggy_msgs::DoggyMove::GAIT_MOVE_1)
    {
        initial_phase << 0.0, 0.5, 0.5, 0.0;
    }
    else if(cmd == doggy_msgs::DoggyMove::GAIT_MOVE_2)
    {
        initial_phase << 0.5, 0.5, 0.5, 0.5;
    }
}

int WalkGait::predict_gait_counter(double dt, int leg_index, bool mode)
{
    int step = dt * 1000.0 / MAIN_UPDATE_INTERVEL_MS;
    int gait_counter_predicted;
    int gait_count_now = gait_counter_(leg_index);
    if( gait_count_now < 0)
    {
        if(gait_count_now + step < 0)
        {
            gait_counter_predicted = gait_count_now + step + gait_spansteps_(leg_index);
        }
        else
        {
            gait_counter_predicted = gait_count_now + step + initial_counter_(leg_index);
        }
    }
    else
    {
        gait_counter_predicted = (gait_count_now + step);
    }
    
    if (mode)
    {
        gait_counter_predicted = gait_counter_predicted % (gait_spansteps_(leg_index));
    }

    return gait_counter_predicted;
}