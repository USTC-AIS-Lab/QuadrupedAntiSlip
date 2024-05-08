#include "mpc_compact_controller.h"


MPCCompactController::MPCCompactController(GaitPtr &gait_ptr): gait_ptr_(gait_ptr)
{
    ros::NodeHandle _nh;
    Ac.setZero();
    q.setZero();
    lb.setZero();
    ub.setZero();
    x_0.setZero();
    A_aug.setZero();
    B_aug.setZero();
    hessian.setZero();
    predicted_trajectory.setZero();
    reference.setZero();

    controller_name =  "mpc_controller";
    _nh.param(controller_name + "/constraint_f_max", F_max, CONSTRAINT_F_MAX);
    _nh.param(controller_name + "/constraint_f_min", F_min, CONSTRAINT_F_MIN);
    _nh.param("friction_mu", friction_mu, FRITION_MU);
    _nh.getParam(controller_name + "/q_weights_0", q_weights_mpc(0));
    _nh.getParam(controller_name + "/q_weights_1", q_weights_mpc(1));
    _nh.getParam(controller_name + "/q_weights_2", q_weights_mpc(2));
    _nh.getParam(controller_name + "/q_weights_3", q_weights_mpc(3));
    _nh.getParam(controller_name + "/q_weights_4", q_weights_mpc(4));
    _nh.getParam(controller_name + "/q_weights_5", q_weights_mpc(5));
    _nh.getParam(controller_name + "/q_weights_6", q_weights_mpc(6));
    _nh.getParam(controller_name + "/q_weights_7", q_weights_mpc(7));
    _nh.getParam(controller_name + "/q_weights_8", q_weights_mpc(8));
    _nh.getParam(controller_name + "/q_weights_9", q_weights_mpc(9));
    _nh.getParam(controller_name + "/q_weights_10", q_weights_mpc(10));
    _nh.getParam(controller_name + "/q_weights_11", q_weights_mpc(11));
    _nh.getParam(controller_name + "/q_weights_12", q_weights_mpc(12));
    _nh.getParam(controller_name + "/k_q_n", k_q_n);
    _nh.getParam(controller_name + "/prediction_dt", prediction_dt);

    _nh.getParam(controller_name + "/r_weights_0", r_weights_mpc(0));
    _nh.getParam(controller_name + "/r_weights_1", r_weights_mpc(1));
    _nh.getParam(controller_name + "/r_weights_2", r_weights_mpc(2));
    _nh.getParam(controller_name + "/r_weights_3", r_weights_mpc(3));
    _nh.getParam(controller_name + "/r_weights_4", r_weights_mpc(4));
    _nh.getParam(controller_name + "/r_weights_5", r_weights_mpc(5));
    _nh.getParam(controller_name + "/r_weights_6", r_weights_mpc(6));
    _nh.getParam(controller_name + "/r_weights_7", r_weights_mpc(7));
    _nh.getParam(controller_name + "/r_weights_8", r_weights_mpc(8));
    _nh.getParam(controller_name + "/r_weights_9", r_weights_mpc(9));
    _nh.getParam(controller_name + "/r_weights_10", r_weights_mpc(10));
    _nh.getParam(controller_name + "/r_weights_11", r_weights_mpc(11));

    _nh.getParam(controller_name + "/roll_delta_max", max_angle(0));
    _nh.getParam(controller_name + "/pitch_delta_max", max_angle(1));
    _nh.getParam(controller_name + "/yaw_delta_max", max_angle(2));

    _nh.getParam(controller_name + "/body_height_max_delta", body_height_max_delta);
    _nh.param(controller_name + "/max_pos_error", max_pos_error, 0.05);

    Eigen::Vector3d kp_lin;
    Eigen::Vector3d kp_ang;
    Eigen::Vector3d ki_lin;
    Eigen::Vector3d ki_ang;
    Eigen::Vector3d lin_vel_p_max;
    Eigen::Vector3d ang_vel_p_max;
    Eigen::Vector3d lin_vel_i_max;
    Eigen::Vector3d ang_vel_i_max;

    _nh.getParam(controller_name + "/kp_angular_x", kp_ang(0));
    _nh.getParam(controller_name + "/kp_angular_y", kp_ang(1));
    _nh.getParam(controller_name + "/kp_angular_z", kp_ang(2));

    _nh.getParam(controller_name + "/kp_linear_x", kp_lin(0));
    _nh.getParam(controller_name + "/kp_linear_y", kp_lin(1));
    _nh.getParam(controller_name + "/kp_linear_z", kp_lin(2));

    _nh.getParam(controller_name + "/ki_angular_x", ki_ang(0));
    _nh.getParam(controller_name + "/ki_angular_y", ki_ang(1));
    _nh.getParam(controller_name + "/ki_angular_z", ki_ang(2));

    _nh.getParam(controller_name + "/ki_linear_x", ki_lin(0));
    _nh.getParam(controller_name + "/ki_linear_y", ki_lin(1));
    _nh.getParam(controller_name + "/ki_linear_z", ki_lin(2));


    _nh.getParam(controller_name + "/lin_vel_p_max_x", lin_vel_p_max(0));
    _nh.getParam(controller_name + "/lin_vel_p_max_y", lin_vel_p_max(1));
    _nh.getParam(controller_name + "/lin_vel_p_max_z", lin_vel_p_max(2));

    _nh.getParam(controller_name + "/ang_vel_p_max_x", ang_vel_p_max(0));
    _nh.getParam(controller_name + "/ang_vel_p_max_y", ang_vel_p_max(1));
    _nh.getParam(controller_name + "/ang_vel_p_max_z", ang_vel_p_max(2));

    _nh.getParam(controller_name + "/lin_vel_i_max_x", lin_vel_i_max(0));
    _nh.getParam(controller_name + "/lin_vel_i_max_y", lin_vel_i_max(1));
    _nh.getParam(controller_name + "/lin_vel_i_max_z", lin_vel_i_max(2));

    _nh.getParam(controller_name + "/ang_vel_i_max_x", ang_vel_i_max(0));
    _nh.getParam(controller_name + "/ang_vel_i_max_y", ang_vel_i_max(1));
    _nh.getParam(controller_name + "/ang_vel_i_max_z", ang_vel_i_max(2));

    pid_lin.set_param(3, kp_lin,
                      ki_lin,
                      Eigen::Vector3d::Zero(),
                      lin_vel_p_max, lin_vel_i_max, Eigen::Vector3d::Zero(), lin_vel_p_max + lin_vel_i_max);
    pid_ang.set_param(3, kp_ang,
                      ki_ang,
                      Eigen::Vector3d::Zero(),
                      ang_vel_p_max, ang_vel_i_max, Eigen::Vector3d::Zero(), ang_vel_p_max + ang_vel_i_max);
    
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM * MPC_COMPACT_PLAN_HORIZON, 1> q_weights_aug;
    Eigen::Matrix<double, NUM_DOF * MPC_COMPACT_PLAN_HORIZON, 1> r_weights_aug;
    for (int i = 0; i < MPC_COMPACT_PLAN_HORIZON; ++i)
    {
        q_weights_aug.segment<MPC_COMPACT_STATE_DIM>(MPC_COMPACT_STATE_DIM * i) = q_weights_mpc;
        if(i == MPC_COMPACT_PLAN_HORIZON - 1)
        {
            q_weights_aug.segment<MPC_COMPACT_STATE_DIM>(MPC_COMPACT_STATE_DIM * i) *= k_q_n;
        }
        r_weights_aug.segment<NUM_DOF>(NUM_DOF * i) = r_weights_mpc;
    }
    Q_aug.diagonal() = q_weights_aug;
    R_aug.diagonal() = r_weights_aug; 

    std::vector<Eigen::Triplet<double>> triplet_list_Ac_static;
    for (int i = 0; i < MPC_COMPACT_PLAN_HORIZON; ++i)
    {
        for (int j = 0; j < NUM_LEG; j++)
        {
            int offset_row = MPC_COMPACT_CONSTRAINT_DIM_u * i + MPC_COMPACT_CONSTRAINT_DIM_PER_LEG * j;
            int offset_col = NUM_DOF * i + NUM_DOF_PER_LEG * j;
            Ac(offset_row + 0, offset_col + 0) = 1.0;
            Ac(offset_row + 0, offset_col + 2) = friction_mu;
            Ac(offset_row + 1, offset_col + 0) = 1.0;
            Ac(offset_row + 1, offset_col + 2) = -friction_mu;
            Ac(offset_row + 2, offset_col + 1) = 1.0;
            Ac(offset_row + 2, offset_col + 2) = friction_mu;
            Ac(offset_row + 3, offset_col + 1) = 1.0;
            Ac(offset_row + 3, offset_col + 2) = -friction_mu;
            Ac(offset_row + 4, offset_col + 2) = 1.0;
        }
    }
    // std::cout << "Ac:" << std::endl;
    // std::cout << Ac << std::endl;
}

void MPCCompactController::update_x_0(DogState &state)
{
    double fitted_yaw_0 = Utils::periodical_clamp(state.attitude.com_euler(2), x_0(2) - PI, x_0(2) + PI, 2 * PI);
    x_0 << state.attitude.com_euler(0), state.attitude.com_euler(1), fitted_yaw_0,
        state.attitude.com_pos(0), state.attitude.com_pos(1), state.attitude.com_pos(2),
        state.attitude.com_ang_vel(0), state.attitude.com_ang_vel(1), state.attitude.com_ang_vel(2),
        state.attitude.com_lin_vel(0), state.attitude.com_lin_vel(1), state.attitude.com_lin_vel(2),
        -GRAVITY_ACCELERATION;  
}

void MPCCompactController::update_reference_trajectory(DogState &state, double prediction_dt)
{
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM * (MPC_COMPACT_PLAN_HORIZON + 1), 1> reference_with_x0;
    reference_with_x0.setZero();
    reference_with_x0.segment(0, MPC_COMPACT_STATE_DIM) = x_0;
    double fitted_yaw_set = Utils::periodical_clamp(state.ctrl.com_euler_set(2) + state.ctrl.yaw_offset_set, x_0(2) - PI, x_0(2) + PI, 2 * PI);
    Eigen::Vector3d pose_error = state.ctrl.com_set - state.attitude.com_pos;
    Eigen::Vector3d euler_error = state.ctrl.com_euler_set + Eigen::Vector3d(0, 0, state.ctrl.yaw_offset_set) - x_0.segment<3>(0);
    euler_error(2) = Utils::periodical_clamp(euler_error(2), -PI, PI, 2 * PI);
    Eigen::Vector3d lin_vel_set_d = state.attitude.com_rot_mat * pid_lin.cal_output(Eigen::Vector3d::Zero(), state.attitude.com_rot_mat.transpose() * pose_error);
    Eigen::Vector3d euler_rate_set_d = state.attitude.com_rot_mat * pid_ang.cal_output(Eigen::Vector3d::Zero(), state.attitude.com_rot_mat.transpose() * euler_error);
    state.ctrl.com_lin_vel_actuated_set = state.ctrl.com_lin_vel_set + lin_vel_set_d;

    for (int time_step = 0; time_step < MPC_COMPACT_PLAN_HORIZON; time_step++)
    {
        Eigen::Vector3d euler_now = reference_with_x0.segment((time_step) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(0);
        Eigen::Vector3d lin_ang_set_d = Utils::euler_to_angular_velocity_mapping(euler_now) * euler_rate_set_d;
        state.ctrl.com_ang_vel_actuated_set = state.ctrl.com_ang_vel_set + lin_ang_set_d;
        
        reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(0) =
            Eigen::Vector3d(state.ctrl.com_euler_set(0), state.ctrl.com_euler_set(1), fitted_yaw_set + state.ctrl.com_ang_vel_set(2) * prediction_dt * (time_step + 1));
        
        if(time_step == 0)
        {
            reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(3) = state.ctrl.com_set;
            if(x_0.segment<3>(3)(0) - state.ctrl.com_set(0) > max_pos_error)
            {
                reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(3)(0) = x_0.segment<3>(3)(0) - max_pos_error;
            }
            if(x_0.segment<3>(3)(0) - state.ctrl.com_set(0) < -max_pos_error)
            {
                reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(3)(0) = x_0.segment<3>(3)(0) + max_pos_error;
            }
            if(x_0.segment<3>(3)(1) - state.ctrl.com_set(1) > max_pos_error)
            {
                reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(3)(1) = x_0.segment<3>(3)(1) - max_pos_error;
            }
            if(x_0.segment<3>(3)(1) - state.ctrl.com_set(1) < -max_pos_error)
            {
                reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(3)(1) = x_0.segment<3>(3)(1) + max_pos_error;
            }

        }
        else
        {
            reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(3) =
                reference_with_x0.segment((time_step)*MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(3) + Utils::euler_to_quat(reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(0) - reference_with_x0.segment((0) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(0)).toRotationMatrix() * state.ctrl.com_lin_vel_set * prediction_dt;
        }
        
        
        reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM)(5) = state.ctrl.com_set[2]; // base height keep constant
        reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(6) = state.ctrl.com_ang_vel_actuated_set;
        reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM).segment<3>(9) = state.ctrl.com_lin_vel_actuated_set;
        reference_with_x0.segment((time_step + 1) * MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM)(12) = - GRAVITY_ACCELERATION;
    }
    reference = reference_with_x0.segment<MPC_COMPACT_STATE_DIM * (MPC_COMPACT_PLAN_HORIZON)>(MPC_COMPACT_STATE_DIM);
}

Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> MPCCompactController::update_A_step(DogState &state, int time_step)
{
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A;
    A.setZero();
    double yaw;
    //yaw = state.attitude.com_euler[2];
    yaw = reference(time_step * MPC_COMPACT_STATE_DIM + 2);
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    // double cos_yaw = cos(state.attitude.com_euler[2]);
    // double sin_yaw = sin(state.attitude.com_euler[2]);
    Eigen::Matrix3d ang_vel_to_rpy_rate;
    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
        -sin_yaw, cos_yaw, 0,
        0, 0, 1;
    A.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    
    A(11, NUM_DOF) = 1;
    return A;
}

Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> MPCCompactController::update_B_step(DogState &state, int time_step)
{
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> B;
    B.setZero();
    Eigen::Matrix3d trunk_inertia_world;
    trunk_inertia_world = Utils::euler_to_quat(predicted_trajectory.segment<3>(time_step * MPC_COMPACT_STATE_DIM)).toRotationMatrix() * state.attitude.trunk_inertia * Utils::euler_to_quat(predicted_trajectory.segment<3>(time_step * MPC_COMPACT_STATE_DIM)).toRotationMatrix().transpose();
    state.ctrl.predicted_foot_poses = gait_ptr_->predict_foot_poses(predicted_trajectory, MPC_COMPACT_STATE_DIM, state, prediction_dt);

    for (int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Vector3d foot_pos_vec;
        if(state.ctrl.predicted_foot_poses.size() > 0)
        {
            foot_pos_vec = state.ctrl.predicted_foot_poses[time_step].block<3, 1>(0, i) - predicted_trajectory.segment<3>(time_step * MPC_COMPACT_STATE_DIM + 3);
        }
        else // predict_foot_poses function is not available
        {
            foot_pos_vec = state.attitude.com_rot_mat * state.attitude.foot_pos_r.block<3, 1>(0, i);
        }
        //Eigen::Vector3d foot_pos_vec = state.attitude.com_rot_mat * state.attitude.foot_pos_r.block<3, 1>(0, i);
        B.block<3, 3>(6, 3 * i) =
        trunk_inertia_world.fullPivHouseholderQr().solve(Utils::skew(foot_pos_vec));
        double predicted_contact;
        // if (time_step == 0)
        // {
        //     predicted_contact = state.contact(i);
        // }
        // else
        // {
        // double mysterious_hyperparameter = 1.5;
        predicted_contact = gait_ptr_->get_contact_plan(time_step * prediction_dt, prediction_dt, i);
        // }

        B.block<3, 3>(9, 3 * i) = (1 / state.attitude.robot_mass) * Eigen::Matrix3d::Identity() * predicted_contact;
    }
    return B;
}

Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> MPCCompactController::discretize_A(Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A)
{
    // Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A_d = Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM>::Identity() + A * prediction_dt;
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A_d = Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM>::Identity() + A * prediction_dt + 0.5 * prediction_dt * prediction_dt * A * A;
    return A_d;
}

Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> MPCCompactController::discretize_B(Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> B, Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A)
{
    // Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> B_d = B * prediction_dt;
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> B_d = (prediction_dt * Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM>::Identity()  + 0.5 * A * prediction_dt * prediction_dt) * B;
    return B_d;
}

void MPCCompactController::update_Ac(DogState &state, double dt)
{
    for (int j = 0; j < NUM_LEG; j++)
    {
        double actual_mu = friction_mu;
        if(state.slip_info.is_probing && j == state.slip_info.probe_leg)
        {
            actual_mu = 0.0; // only provide z force by mpc during probe.
        }
        else if (state.slip_info.slip_state(j))
        {
            actual_mu = friction_mu * 0.1; // this param is important
        }
        for (int i = 0; i < MPC_COMPACT_PLAN_HORIZON; ++i)
        {
            int offset_row = MPC_COMPACT_CONSTRAINT_DIM_u * i + MPC_COMPACT_CONSTRAINT_DIM_PER_LEG * j;
            int offset_col = NUM_DOF * i + NUM_DOF_PER_LEG * j;
            Ac(offset_row + 0, offset_col + 2) = actual_mu;
            Ac(offset_row + 1, offset_col + 2) = -actual_mu;
            Ac(offset_row + 2, offset_col + 2) = actual_mu;
            Ac(offset_row + 3, offset_col + 2) = -actual_mu;
        }
    }
    // std::cout << "Ac:" << std::endl;
    // std::cout << Ac << std::endl;
}

void MPCCompactController::update_AB_aug(DogState &state, double mpc_dt)
{
    for(int time_step = 0; time_step < MPC_COMPACT_PLAN_HORIZON; ++time_step)
    {
        Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A = update_A_step(state, time_step);
        Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A_d = discretize_A(A);
        Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> B = update_B_step(state, time_step);
        Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> B_d = discretize_B(B, A);

        if(time_step == 0)
        {
            A_aug.block<MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM>(MPC_COMPACT_STATE_DIM * time_step, 0) = A_d;
        }
        else
        {
            A_aug.block<MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM>(MPC_COMPACT_STATE_DIM * time_step, 0) = A_d * A_aug.block<MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM>(MPC_COMPACT_STATE_DIM * (time_step - 1), 0);
        }

        for(int i = 0; i < time_step + 1; ++i)
        {
            if(i == time_step)
            {
                B_aug.block<MPC_COMPACT_STATE_DIM, NUM_DOF>(MPC_COMPACT_STATE_DIM * time_step, NUM_DOF * i) = B_d;
            }
            else
            {
                B_aug.block<MPC_COMPACT_STATE_DIM, NUM_DOF>(MPC_COMPACT_STATE_DIM * time_step, NUM_DOF * i) = A_d * B_aug.block<MPC_COMPACT_STATE_DIM, NUM_DOF>(MPC_COMPACT_STATE_DIM * (time_step - 1), NUM_DOF * i);
            }
        }

    }
    
    // std::cout << "A_aug: " << std::endl;
    // std::cout << A_aug << std::endl;

    // std::cout << "B_aug: " << std::endl;
    // std::cout << B_aug << std::endl;

}

void MPCCompactController::update_hessian(DogState &state, double dt)
{
    hessian = 0.5 * (B_aug.transpose() * Q_aug * B_aug + R_aug.toDenseMatrix());
}

void MPCCompactController::update_bound(DogState &state)
{
    lb.setZero();
    ub.setZero();
    for (int time_step = 0; time_step < MPC_COMPACT_PLAN_HORIZON; time_step++)
    {
        /*** Control Constraints ***/
        for (int j = 0; j < NUM_LEG; ++j)
        {
            double predicted_contact;
            if (time_step == 0)
            {
                predicted_contact = state.contact(j);
            }
            else
            {
                predicted_contact = gait_ptr_->get_contact_plan(time_step * prediction_dt, prediction_dt, j) > 0.0;
            }
            // std::cout << time_step <<", " << j << ": " << predicted_contact << std::endl;
            int offset = time_step * MPC_COMPACT_CONSTRAINT_DIM_u + j * MPC_COMPACT_CONSTRAINT_DIM_PER_LEG;
            double f_min_actual = F_min;
            double f_max_actual = F_max;
            if(state.slip_info.is_probing && j == state.slip_info.probe_leg)
            {
                f_min_actual = state.slip_info.probe_force(2);
                f_max_actual = state.slip_info.probe_force(2);
            }


            lb.segment<MPC_COMPACT_CONSTRAINT_DIM_PER_LEG>(offset) << 0, -OsqpEigen::INFTY, 0, -OsqpEigen::INFTY, f_min_actual * predicted_contact;
            ub.segment<MPC_COMPACT_CONSTRAINT_DIM_PER_LEG>(offset) << OsqpEigen::INFTY, 0, OsqpEigen::INFTY, 0, f_max_actual * predicted_contact;
        }
    }
    // std::cout << "lb: " << std::endl;
    // std::cout << lb << std::endl;
    // std::cout << "ub: " << std::endl;
    // std::cout << ub << std::endl;
}

void MPCCompactController::update_q(DogState &state)
{
    q = B_aug.transpose() * Q_aug * (A_aug * x_0 - reference);
}

void MPCCompactController::QP_solve()
{
    auto t1 = std::chrono::high_resolution_clock::now();
    // OsqpEigen::Solver solver;
    // osqp_mkl_test();

    Eigen::SparseMatrix<double> Ac_sparse(MPC_COMPACT_CONSTRAINT_DIM_u * MPC_COMPACT_PLAN_HORIZON, NUM_DOF * MPC_COMPACT_PLAN_HORIZON);
    Eigen::SparseMatrix<double> hessian_sparse(NUM_DOF * MPC_COMPACT_PLAN_HORIZON, NUM_DOF * MPC_COMPACT_PLAN_HORIZON);
    Ac_sparse = Ac.sparseView();
    hessian_sparse = hessian.sparseView();

    if (!solver.isInitialized())
    {
        
        solver.settings()->setVerbosity(true); 
        solver.settings()->setWarmStart(true);
        solver.settings()->setRelativeTolerance(1e-4);
        solver.settings()->setAbsoluteTolerance(1e-4);
        solver.settings()->setLinearSystemSolver(QDLDL_SOLVER); // QDLDL_SOLVER MKL_PARDISO_SOLVER
        solver.data()->setNumberOfVariables(NUM_DOF * MPC_COMPACT_PLAN_HORIZON);
        solver.data()->setNumberOfConstraints(MPC_COMPACT_CONSTRAINT_DIM_u * MPC_COMPACT_PLAN_HORIZON);
        solver.data()->setLinearConstraintsMatrix(Ac_sparse);
        solver.data()->setHessianMatrix(hessian_sparse);
        solver.data()->setGradient(q);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);
        solver.initSolver();
        // mkl_set_num_threads(4);
    }
    else
    {
        solver.updateLinearConstraintsMatrix(Ac_sparse);
        solver.updateHessianMatrix(hessian_sparse);
        solver.updateGradient(q);
        solver.updateBounds(lb, ub);

    }
    
    auto t2 = std::chrono::high_resolution_clock::now();
    solver.solveProblem();
    auto t3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
    std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
    solution = solver.getSolution();
    // std::cout << "mpc solve time: " << ms_double_2.count() << "ms" << std::endl
    //           << std::endl;
    // predicted_trajectory = solution.segment<MPC_COMPACT_STATE_DIM * (MPC_COMPACT_PLAN_HORIZON + 1)>(0);

}

void MPCCompactController::update_prediction_trajactory()
{
    predicted_trajectory = A_aug * x_0 + B_aug * solution;
}

Eigen::Matrix<double, 3, NUM_LEG> MPCCompactController::get_control(DogState &state)
{
    Eigen::Matrix<double, 3, NUM_LEG> control;
    control.setZero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (!isnan(solution.segment<3>(i * 3).norm()))
        {
            Eigen::Vector3d solution_force = solution.segment<3>(i * 3);
            solution_force += Eigen::Vector3d(0, 0, 0.5);
            control.block<3, 1>(0, i) = state.attitude.com_rot_mat.transpose() * solution_force;
        }
    }
    return control;
}


Eigen::Matrix<double, 3, NUM_LEG> MPCCompactController::compute_ground_reaction_force(DogState &state, double dt)
{

    auto t1 = std::chrono::high_resolution_clock::now();
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    update_x_0(state);
    auto t2 = std::chrono::high_resolution_clock::now();
    update_reference_trajectory(state, prediction_dt);
    auto t3 = std::chrono::high_resolution_clock::now();
    update_Ac(state, prediction_dt);
    update_AB_aug(state, prediction_dt);
    update_hessian(state, prediction_dt);
    auto t4 = std::chrono::high_resolution_clock::now();
    update_bound(state);
    auto t5 = std::chrono::high_resolution_clock::now();
    update_q(state);
    auto t6 = std::chrono::high_resolution_clock::now();
    QP_solve();
    foot_forces_grf = get_control(state);
    update_prediction_trajactory();
    
    state.ctrl.mpc_reference = get_reference_trajactory();
    state.ctrl.mpc_prediction = get_prediction_trajactory();
    auto t7 = std::chrono::high_resolution_clock::now();
    std::cout << "total solve time: " << ((std::chrono::duration<double, std::milli>)(t7 - t1)).count() << "ms" << std::endl;
    return foot_forces_grf;
}

geometry_msgs::PoseArray MPCCompactController::get_reference_trajactory()
{
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = ros::Time::now();
    for(int i = 0; i < (MPC_COMPACT_PLAN_HORIZON); ++i)
    {
        geometry_msgs::Pose pose;
        pose.position.x = reference(i * MPC_COMPACT_STATE_DIM + 3);
        pose.position.y = reference(i * MPC_COMPACT_STATE_DIM + 4);
        pose.position.z = reference(i * MPC_COMPACT_STATE_DIM + 5);
        Eigen::Quaterniond quat = Utils::euler_to_quat(reference.segment<3>(i * MPC_COMPACT_STATE_DIM + 0));
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        pose_array.poses.emplace_back(pose);
    }
    return pose_array;
}

geometry_msgs::PoseArray MPCCompactController::get_prediction_trajactory()
{
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = ros::Time::now();
    for(int i = 0; i < (MPC_COMPACT_PLAN_HORIZON); ++i)
    {
        geometry_msgs::Pose pose;
        pose.position.x = predicted_trajectory(i * MPC_COMPACT_STATE_DIM + 3);
        pose.position.y = predicted_trajectory(i * MPC_COMPACT_STATE_DIM + 4);
        pose.position.z = predicted_trajectory(i * MPC_COMPACT_STATE_DIM + 5);
        Eigen::Quaterniond quat = Utils::euler_to_quat(predicted_trajectory.segment<3>(i * MPC_COMPACT_STATE_DIM + 0));
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        pose_array.poses.emplace_back(pose);
    }
    return pose_array;
}


