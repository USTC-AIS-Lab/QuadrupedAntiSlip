#include "mpc_controller.h"

MPCController::MPCController(GaitPtr &gait_ptr): gait_ptr_(gait_ptr)
{
    ros::NodeHandle _nh;
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

    _nh.getParam(controller_name + "/roll_delta_max", max_angle(0));
    _nh.getParam(controller_name + "/pitch_delta_max", max_angle(1));
    _nh.getParam(controller_name + "/yaw_delta_max", max_angle(2));

    _nh.getParam(controller_name + "/body_height_max_delta", body_height_max_delta);
    _nh.param(controller_name + "/save_log", is_save_log, false);
    _nh.param(controller_name + "/max_pos_error", max_pos_error, 0.05);
    

    if(is_save_log)
    {
        std::cout << "saving running log in MPC." << std::endl;
        log_file = std::ofstream("/root/A1_ctrl_ws/src/doggy_control/src/test/data/draw_mpc.csv");
    }


    pid_lin.set_param(3, kp_lin,
                      ki_lin,
                      Eigen::Vector3d::Zero(),
                      lin_vel_p_max, lin_vel_i_max, Eigen::Vector3d::Zero(), lin_vel_p_max + lin_vel_i_max);
    pid_ang.set_param(3, kp_ang,
                      ki_ang,
                      Eigen::Vector3d::Zero(),
                      ang_vel_p_max, ang_vel_i_max, Eigen::Vector3d::Zero(), ang_vel_p_max + ang_vel_i_max);
    Q_mpc.diagonal() = q_weights_mpc;
    R_mpc.diagonal() = r_weights_mpc;

    P_sparse = Eigen::SparseMatrix<double>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON);
    Ac_sparse = Eigen::SparseMatrix<double>((MPC_STATE_DIM) * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON);

    q.setZero();
    lb.setZero();
    ub.setZero();
    // linear_constraints_dense.setZero();
    // hessian_dense P is constant:[Q, Q, ..., Q, R, R, ..., R].
    // gredient q depends on x_k^r (reference), and is updated in each loop.
    std::vector<Eigen::Triplet<double>> triplet_list_p;
    for (int i = 0; i < MPC_PLAN_HORIZON + 1; ++i)
    {
        std::vector<Eigen::Triplet<double>> new_triplet_list;
        if (i < MPC_PLAN_HORIZON)
        {
            new_triplet_list = Utils::matrix_to_none_zero_tripletList(Q_mpc, MPC_STATE_DIM * i, MPC_STATE_DIM * i);
        }
        else
        {
            new_triplet_list = Utils::matrix_to_none_zero_tripletList(Q_mpc * k_q_n, MPC_STATE_DIM * i, MPC_STATE_DIM * i);
        }
        triplet_list_p.insert(triplet_list_p.end(), new_triplet_list.begin(), new_triplet_list.end());
    }
    for (int i = 0; i < MPC_PLAN_HORIZON; ++i)
    {
        std::vector<Eigen::Triplet<double>> new_triplet_list = Utils::matrix_to_none_zero_tripletList(R_mpc, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * i, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * i);
        triplet_list_p.insert(triplet_list_p.end(), new_triplet_list.begin(), new_triplet_list.end());
    }
    P_sparse.setFromTriplets(triplet_list_p.begin(), triplet_list_p.end());
    for (int i = 0; i < (MPC_PLAN_HORIZON + 1); ++i)
    {
        std::vector<Eigen::Triplet<double>> new_triplet_list = Utils::matrix_to_none_zero_tripletList(-Eigen::MatrixXd::Identity(MPC_STATE_DIM, MPC_STATE_DIM), MPC_STATE_DIM * i, MPC_STATE_DIM * i);
        triplet_list_Ac_static.insert(triplet_list_Ac_static.end(), new_triplet_list.begin(), new_triplet_list.end());
        // linear_constraints_dense.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, MPC_STATE_DIM * i) = Eigen::MatrixXd::Identity(MPC_STATE_DIM, MPC_STATE_DIM);
    }
    for (int i = 0; i < MPC_PLAN_HORIZON; ++i)
    {
        for (int j = 0; j < NUM_LEG; j++)
        {
            int offset_row = MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CONSTRAINT_DIM_x * MPC_PLAN_HORIZON + MPC_CONSTRAINT_DIM_u * i + MPC_CONSTRAINT_DIM_PER_LEG * j;
            int offset_col = MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * i + NUM_DOF_PER_LEG * j;
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 0, offset_col + 0, 1.0));
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 0, offset_col + 2, friction_mu));
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 1, offset_col + 0, 1.0));
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 1, offset_col + 2, -friction_mu));
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 2, offset_col + 1, 1.0));
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 2, offset_col + 2, friction_mu));
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 3, offset_col + 1, 1.0));
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 3, offset_col + 2, -friction_mu));
            triplet_list_Ac_static.emplace_back(Eigen::Triplet<double>(offset_row + 4, offset_col + 2, 1.0));
        }
    }
    Ac_sparse.setFromTriplets(triplet_list_Ac_static.begin(), triplet_list_Ac_static.end());
    // std::cout << "P_sparse:" << std::endl;
    // std::cout << P_sparse << std::endl;
    // std::cout << "static Ac_sparse:" << std::endl;
    // std::cout << Ac_sparse << std::endl;
}

Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> MPCController::update_A_step(DogState &state, int time_step)
{
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A;
    A.setZero();
    double yaw;
    //yaw = state.attitude.com_euler[2];
    yaw = reference(time_step * MPC_STATE_DIM + 2);
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

Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> MPCController::update_B_step(DogState &state, int time_step)
{
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B;
    B.setZero();
    Eigen::Matrix3d trunk_inertia_world;
    trunk_inertia_world = Utils::euler_to_quat(predicted_trajectory.segment<3>(time_step * MPC_STATE_DIM)).toRotationMatrix() * state.attitude.trunk_inertia * Utils::euler_to_quat(predicted_trajectory.segment<3>(time_step * MPC_STATE_DIM)).toRotationMatrix().transpose();
    

    for (int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Vector3d foot_pos_vec;
        // if(state.ctrl.predicted_foot_poses.size() > 0)
        // {
        //     foot_pos_vec = state.ctrl.predicted_foot_poses[time_step].block<3, 1>(0, i) - predicted_trajectory.segment<3>(time_step * MPC_STATE_DIM + 3);
        // }
        // else // predict_foot_poses function is not available
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

Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> MPCController::discretize_A(Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A)
{
    // Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A * prediction_dt;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A * prediction_dt + 0.5 * prediction_dt * prediction_dt * A * A;
    return A_d;
}

Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> MPCController::discretize_B(Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B, Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A)
{
    // Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_d = B * prediction_dt;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_d = (prediction_dt * Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity()  + 0.5 * A * prediction_dt * prediction_dt) * B;
    return B_d;
}


void MPCController::update_Ac(DogState &state, double dt)
{
    auto t1 = std::chrono::high_resolution_clock::now();
    triplet_list_Ac = triplet_list_Ac_static;
    state.ctrl.predicted_foot_poses = gait_ptr_->predict_foot_poses(predicted_trajectory, MPC_STATE_DIM, state, prediction_dt);
    for (int time_step = 0; time_step < MPC_PLAN_HORIZON; time_step++)
    {
        Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A = update_A_step(state, time_step);
        Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_d = discretize_A(A);
        Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B = update_B_step(state, time_step);
        Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_d = discretize_B(B, A);

        std::vector<Eigen::Triplet<double>> new_triplet_list = Utils::matrix_to_none_zero_tripletList(A_d, MPC_STATE_DIM * (time_step + 1), MPC_STATE_DIM * time_step);
        triplet_list_Ac.insert(triplet_list_Ac.end(), new_triplet_list.begin(), new_triplet_list.end());
        new_triplet_list = Utils::matrix_to_none_zero_tripletList(B_d, MPC_STATE_DIM * (time_step + 1), MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * time_step);
        triplet_list_Ac.insert(triplet_list_Ac.end(), new_triplet_list.begin(), new_triplet_list.end());
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    Ac_sparse.setFromTriplets(triplet_list_Ac.begin(), triplet_list_Ac.end());

    auto t3 = std::chrono::high_resolution_clock::now();

    for (int j = 0; j < NUM_LEG; j++)
    {
        double actual_mu = friction_mu;
        if(state.slip_info.is_probing && j == state.slip_info.probe_leg)
        {
            actual_mu = 0.0; // only provide z force by mpc during probe.
        }
        else if (state.slip_info.slip_state(j))
        {
            actual_mu = friction_mu * 0.5; // this param is important
        }
        for (int i = 0; i < MPC_PLAN_HORIZON; ++i)
        {
            int offset_row = MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CONSTRAINT_DIM_x * MPC_PLAN_HORIZON + MPC_CONSTRAINT_DIM_u * i + MPC_CONSTRAINT_DIM_PER_LEG * j;
            int offset_col = MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * i + NUM_DOF_PER_LEG * j;
            Ac_sparse.coeffRef(offset_row + 0, offset_col + 2) = actual_mu;
            Ac_sparse.coeffRef(offset_row + 1, offset_col + 2) = -actual_mu;
            Ac_sparse.coeffRef(offset_row + 2, offset_col + 2) = actual_mu;
            Ac_sparse.coeffRef(offset_row + 3, offset_col + 2) = -actual_mu;
        }
    }
    
    // std::cout << "Ac time1: " << ((std::chrono::duration<double, std::milli>)(t2 - t1)).count() << "ms" << std::endl;
    // std::cout << "Ac time2: " << ((std::chrono::duration<double, std::milli>)(t3 - t2)).count() << "ms" << std::endl;
    // std::cout << "Ac_sparse:" << std::endl;
    // std::cout << Ac_sparse << std::endl;
}

void MPCController::update_bound(DogState &state)
{
    lb.setZero();
    ub.setZero();
    lb.segment<MPC_STATE_DIM>(0) = -x_0;
    ub.segment<MPC_STATE_DIM>(0) = -x_0;
    // std::cout << "lb.segment<MPC_STATE_DIM>(0)" << std::endl;
    // std::cout << lb.segment<MPC_STATE_DIM>(0) << std::endl;
    // std::cout << "ub.segment<MPC_STATE_DIM>(0)" << std::endl;
    // std::cout << ub.segment<MPC_STATE_DIM>(0) << std::endl;
    for (int time_step = 0; time_step < MPC_PLAN_HORIZON; time_step++)
    {
        /*** Transition Constraints ***/
        lb.segment<MPC_STATE_DIM>((time_step + 1) * MPC_STATE_DIM).setZero();
        ub.segment<MPC_STATE_DIM>((time_step + 1) * MPC_STATE_DIM).setZero();
        // lb((time_step + 1) * MPC_STATE_DIM + 11) = 1.0;
        // ub((time_step + 1) * MPC_STATE_DIM + 11) = 1.0;
        /*** State Constraints ***/
        // leave this part for future work
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
            int offset = (MPC_PLAN_HORIZON + 1) * MPC_STATE_DIM + MPC_PLAN_HORIZON * MPC_CONSTRAINT_DIM_x + time_step * MPC_CONSTRAINT_DIM_u + j * MPC_CONSTRAINT_DIM_PER_LEG;
            double f_min_actual = F_min;
            double f_max_actual = F_max;
            if(state.slip_info.is_probing && j == state.slip_info.probe_leg)
            {
                f_min_actual = state.slip_info.probe_force(2);
                f_max_actual = state.slip_info.probe_force(2);
            }


            lb.segment<MPC_CONSTRAINT_DIM_PER_LEG>(offset) << 0, -OsqpEigen::INFTY, 0, -OsqpEigen::INFTY, f_min_actual * predicted_contact;
            ub.segment<MPC_CONSTRAINT_DIM_PER_LEG>(offset) << OsqpEigen::INFTY, 0, OsqpEigen::INFTY, 0, f_max_actual * predicted_contact;
            // lb.segment<MPC_CONSTRAINT_DIM_PER_LEG>(offset) << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
            // ub.segment<MPC_CONSTRAINT_DIM_PER_LEG>(offset) << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;
            // lb.segment<MPC_CONSTRAINT_DIM_PER_LEG>(offset) << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, F_min * predicted_contact;
            // ub.segment<MPC_CONSTRAINT_DIM_PER_LEG>(offset) << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, F_max * predicted_contact;
        }
    }

    // for(int i = 0; i < lb.rows(); ++i)
    // {
    //     lb(i) = -OsqpEigen::INFTY;
    //     ub(i) = OsqpEigen::INFTY;
    // }

    // std::cout<<"lb:"<< std::endl;
    // std::cout<<lb<< std::endl;
    // std::cout<<"ub:"<< std::endl;
    // std::cout<<ub<< std::endl;
}

void MPCController::update_x_0(DogState &state)
{
    double fitted_yaw_0 = Utils::periodical_clamp(state.attitude.com_euler(2), x_0(2) - PI, x_0(2) + PI, 2 * PI);
    fitted_yaw_0 = Utils::periodical_clamp(fitted_yaw_0, - PI, + PI, 2 * PI);
    x_0 << state.attitude.com_euler(0), state.attitude.com_euler(1), fitted_yaw_0,
        state.attitude.com_pos(0), state.attitude.com_pos(1), state.attitude.com_pos(2),
        state.attitude.com_ang_vel(0), state.attitude.com_ang_vel(1), state.attitude.com_ang_vel(2),
        state.attitude.com_lin_vel(0), state.attitude.com_lin_vel(1), state.attitude.com_lin_vel(2),
        -GRAVITY_ACCELERATION;  
}

void MPCController::update_q(DogState &state)
{
    q.setZero();
    // for (int time_step = 0; time_step < MPC_PLAN_HORIZON + 1; time_step++)
    // {
    //     q.segment<MPC_STATE_DIM>(time_step * MPC_STATE_DIM) = -1 * Q_mpc * reference.segment<MPC_STATE_DIM>(time_step * MPC_STATE_DIM);
    // }
    Eigen::Matrix<double, MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, 1> reference_no_input;
    reference_no_input.setZero();
    reference_no_input.segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0) = reference;
    q.segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0) = -(P_sparse * reference_no_input).segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0);
    // std::cout << "correct q:" << std::endl;
    // std::cout << q << std::endl;
    // std::cout<<"q: "<<std::endl;
    // std::cout<<q<<std::endl;
    // q.setZero();
}

void MPCController::update_reference_trajectory(DogState &state, double prediction_dt)
{
    reference.segment(0, MPC_STATE_DIM) = x_0;
    double fitted_yaw_set = Utils::periodical_clamp(state.ctrl.com_euler_set(2) + state.ctrl.yaw_offset_set, x_0(2) - PI, x_0(2) + PI, 2 * PI);
    Eigen::Vector3d pose_error = state.ctrl.com_set - state.attitude.com_pos;
    Eigen::Vector3d euler_error = state.ctrl.com_euler_set + Eigen::Vector3d(0, 0, state.ctrl.yaw_offset_set) - x_0.segment<3>(0);
    euler_error(2) = Utils::periodical_clamp(euler_error(2), -PI, PI, 2 * PI);
    Eigen::Vector3d lin_vel_set_d = state.attitude.com_rot_mat * pid_lin.cal_output(Eigen::Vector3d::Zero(), state.attitude.com_rot_mat.transpose() * pose_error);
    Eigen::Vector3d euler_rate_set_d = state.attitude.com_rot_mat * pid_ang.cal_output(Eigen::Vector3d::Zero(), state.attitude.com_rot_mat.transpose() * euler_error);
    state.ctrl.com_lin_vel_actuated_set = state.ctrl.com_lin_vel_set + lin_vel_set_d;

    for (int time_step = 0; time_step < MPC_PLAN_HORIZON; time_step++)
    {
        Eigen::Vector3d euler_now = reference.segment((time_step) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(0);
        Eigen::Vector3d lin_ang_set_d = Utils::euler_to_angular_velocity_mapping(euler_now) * euler_rate_set_d;
        state.ctrl.com_ang_vel_actuated_set = state.ctrl.com_ang_vel_set + lin_ang_set_d;
        
        reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(0) =
            Eigen::Vector3d(state.ctrl.com_euler_set(0), state.ctrl.com_euler_set(1), fitted_yaw_set + state.ctrl.com_ang_vel_set(2) * prediction_dt * (time_step + 1));
        
        if(time_step == 0)
        {
            reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(3) = state.ctrl.com_set;
            if(x_0.segment<3>(3)(0) - state.ctrl.com_set(0) > max_pos_error)
            {
                reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(3)(0) = x_0.segment<3>(3)(0) - max_pos_error;
            }
            if(x_0.segment<3>(3)(0) - state.ctrl.com_set(0) < -max_pos_error)
            {
                reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(3)(0) = x_0.segment<3>(3)(0) + max_pos_error;
            }
            if(x_0.segment<3>(3)(1) - state.ctrl.com_set(1) > max_pos_error)
            {
                reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(3)(1) = x_0.segment<3>(3)(1) - max_pos_error;
            }
            if(x_0.segment<3>(3)(1) - state.ctrl.com_set(1) < -max_pos_error)
            {
                reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(3)(1) = x_0.segment<3>(3)(1) + max_pos_error;
            }

        }
        else
        {
            reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(3) =
                reference.segment((time_step)*MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(3) + Utils::euler_to_quat(reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(0) - reference.segment((0) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(0)).toRotationMatrix() * state.ctrl.com_lin_vel_actuated_set * prediction_dt;
        }
        
        reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM)(5) = state.ctrl.com_set[2]; // base height keep constant
        reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(6) = state.ctrl.com_ang_vel_actuated_set;
        reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM).segment<3>(9) = state.ctrl.com_lin_vel_actuated_set;
        reference.segment((time_step + 1) * MPC_STATE_DIM, MPC_STATE_DIM)(12) = - GRAVITY_ACCELERATION;
    }
}

void MPCController::QP_solve()
{
    auto t1 = std::chrono::high_resolution_clock::now();
    // OsqpEigen::Solver solver;
    // osqp_mkl_test();
    if (!solver.isInitialized())
    {
        
        solver.settings()->setVerbosity(false); 
        solver.settings()->setWarmStart(true);
        solver.settings()->setRelativeTolerance(1e-5);
        solver.settings()->setAbsoluteTolerance(1e-5);
        solver.settings()->setMaxIteration(125);
        solver.settings()->setLinearSystemSolver(QDLDL_SOLVER); // QDLDL_SOLVER MKL_PARDISO_SOLVER
        solver.data()->setNumberOfVariables(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON);
        solver.data()->setNumberOfConstraints((MPC_STATE_DIM) * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON);
        solver.data()->setLinearConstraintsMatrix(Ac_sparse);
        solver.data()->setHessianMatrix(P_sparse);
        solver.data()->setGradient(q);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);
        solver.initSolver();
        // mkl_set_num_threads(4);
    }
    else
    {
        solver.updateLinearConstraintsMatrix(Ac_sparse);
        solver.updateGradient(q);
        solver.updateBounds(lb, ub);

    }
    
    auto t2 = std::chrono::high_resolution_clock::now();
    solver.solveProblem();
    auto t3 = std::chrono::high_resolution_clock::now();
    solution = solver.getSolution();
    // std::cout << "mpc solve time: " << ms_double_2.count() << "ms" << std::endl
    //           << std::endl;
    predicted_trajectory = solution.segment<MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1)>(0);

    auto t4 = std::chrono::high_resolution_clock::now();

    // std::cout << "*** in QP solve ***"<< std::endl;

    // std::cout << "qp solve time: " << ((std::chrono::duration<double, std::milli>)(t3 - t2)).count() << "ms" << std::endl;

    // std::cout << "qp prepare time: " << ((std::chrono::duration<double, std::milli>)(t2 - t1)).count() << "ms" << std::endl;

    // std::cout << "reset trajectory time: " << ((std::chrono::duration<double, std::milli>)(t4 - t3)).count() << "ms" << std::endl;
    // std::cout << "***************"<< std::endl;

    // std::cout << "reference:" <<std::endl;
    // std::cout << reference <<std::endl;
    // std::cout << "predicted_trajectory:" <<std::endl;
    // std::cout << predicted_trajectory <<std::endl;
    // std::cout << "solution:" <<std::endl;
    // std::cout << solution <<std::endl;

    // std::cout << "solution * Ac - lb:" <<std::endl;
    // std::cout <<  Ac_sparse * solution - lb <<std::endl;

    // std::cout << "solution * Ac - ub:" <<std::endl;
    // std::cout <<  Ac_sparse * solution - ub <<std::endl;
}

Eigen::Matrix<double, 3, NUM_LEG> MPCController::get_control(DogState &state)
{
    Eigen::Matrix<double, 3, NUM_LEG> control;
    control.setZero();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        if (!isnan(solution.segment<3>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + i * 3).norm()))
        {
            Eigen::Vector3d solution_force = solution.segment<3>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + i * 3);
            solution_force += Eigen::Vector3d(0, 0, 0.5);
            control.block<3, 1>(0, i) = state.attitude.com_rot_mat.transpose() * solution_force;
        }
    }
    return control;
}

Eigen::Matrix<double, 3, NUM_LEG> MPCController::compute_ground_reaction_force(DogState &state, double dt)
{

    auto t1 = std::chrono::high_resolution_clock::now();
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    update_x_0(state);
    auto t2 = std::chrono::high_resolution_clock::now();
    update_reference_trajectory(state, prediction_dt);
    auto t3 = std::chrono::high_resolution_clock::now();
    update_Ac(state, prediction_dt);
    auto t4 = std::chrono::high_resolution_clock::now();
    update_bound(state);
    auto t5 = std::chrono::high_resolution_clock::now();
    update_q(state);
    auto t6 = std::chrono::high_resolution_clock::now();
    QP_solve();
    foot_forces_grf = get_control(state);
    auto t7 = std::chrono::high_resolution_clock::now();
    debug(state, dt);
    auto t8 = std::chrono::high_resolution_clock::now();

    // std::cout << "=========" << std::endl;
    // std::cout << "update_x_0 time: " << ((std::chrono::duration<double, std::milli>)(t2 - t1)).count() << "ms" << std::endl;
    // std::cout << "update_reference_trajectory time: " << ((std::chrono::duration<double, std::milli>)(t3 - t2)).count() << "ms" << std::endl;
    // std::cout << "update_Ac time: " << ((std::chrono::duration<double, std::milli>)(t4 - t3)).count() << "ms" << std::endl;
    // std::cout << "update_bound time: " << ((std::chrono::duration<double, std::milli>)(t5 - t4)).count() << "ms" << std::endl;
    // std::cout << "update_q time: " << ((std::chrono::duration<double, std::milli>)(t6 - t5)).count() << "ms" << std::endl;
    // std::cout << "QP_solve time: " << ((std::chrono::duration<double, std::milli>)(t7 - t6)).count() << "ms" << std::endl;
    // std::cout << "debug time: " << ((std::chrono::duration<double, std::milli>)(t8 - t7)).count() << "ms" << std::endl;
    // std::cout << "total solve time: " << ((std::chrono::duration<double, std::milli>)(t8 - t1)).count() << "ms" << std::endl;
    // std::cout<<"foot_forces_grf:"<< std::endl;
    // std::cout<<foot_forces_grf<< std::endl;
    
    return foot_forces_grf;
}

void MPCController::debug_cout_trajectory(int value_index)
{
    std::string value_name_list[6] = {"roll", "pitch", "yaw", "x", "y", "z"};
    int debug_index = value_index;
    std::cout << " ====== debug trajectory of " << value_name_list[value_index] << " ===== " << std::endl;
    std::cout << "reference:" << std::endl;
    for (int i = 0; i < MPC_PLAN_HORIZON + 1; ++i)
    {
        std::cout << reference(MPC_STATE_DIM * i + debug_index) << " ";
    }
    std::cout << std::endl;
    for (int i = 0; i < MPC_PLAN_HORIZON + 1; ++i)
    {
        std::cout << reference(MPC_STATE_DIM * i + debug_index + 6) << " ";
    }
    std::cout << std::endl;
    std::cout << "predicted:" << std::endl;
    for (int i = 0; i < MPC_PLAN_HORIZON + 1; ++i)
    {
        std::cout << predicted_trajectory(MPC_STATE_DIM * i + debug_index) << " ";
    }
    std::cout << std::endl;
    for (int i = 0; i < MPC_PLAN_HORIZON + 1; ++i)
    {
        std::cout << predicted_trajectory(MPC_STATE_DIM * i + debug_index + 6) << " ";
    }
    std::cout << std::endl;

    if(value_index == 5) // for z force
    {
        for (int i = 0; i < MPC_PLAN_HORIZON; ++i)
        {
            double z_force_sum = 0;
            for(int k = 0; k < NUM_LEG; ++k)
            {
                z_force_sum += solution.segment<3>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * i + k * 3)(2);
            }
            std::cout << z_force_sum << " ";
        }
        std::cout << std::endl;
    }
}

void MPCController::debug_loss_analyse()
{
    std::cout << " ===== loss analyse =====" << std::endl;
    Eigen::Matrix<double, MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, 1> reference_no_input;
    Eigen::Matrix<double, MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, 1> solution_no_input;
    reference_no_input.setZero();
    solution_no_input.setZero();
    reference_no_input.segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0) = reference;
    solution_no_input.segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0) = solution.segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0);
    Eigen::Matrix<double, MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, 1> error_vector = solution - reference_no_input;
    double loss = 0.5 * (error_vector.transpose() * P_sparse * error_vector)(0); // + q.transpose() * solution;
    double loss2 = 0.5 * (solution_no_input.transpose() * P_sparse * solution_no_input + reference_no_input.transpose() * P_sparse * reference_no_input)(0) + q.transpose() * solution;
    double loss3 = 0.5 * (solution_no_input.transpose() * P_sparse * solution_no_input + reference_no_input.transpose() * P_sparse * reference_no_input)(0) - reference_no_input.transpose() * P_sparse * solution_no_input;
    // std::cout << "loss1:" << loss << std::endl;
    // std::cout << "loss2:" << loss2 << std::endl;
    std::cout << "loss3:" << loss3 << std::endl;
    // std::cout << "error_vec: " << std::endl;
    // for (int i = 0; i < error_vector.rows(); ++i)
    // {
    //     if (i < MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1))
    //     {
    //         std::cout << i + 1 << ", " << i % MPC_STATE_DIM + 1 << ": " << error_vector(i) << std::endl;
    //     }
    //     else
    //     {
    //         std::cout << i + 1 << ", " << (i - MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1)) % NUM_DOF + 1 << ": " << error_vector(i) << std::endl;
    //     }
    // }
    // std::cout << "loss_vec: " << std::endl;
    // Eigen::Matrix<double, MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, 1> onehot_temp;
    // double loss_sum = 0;
    // for (int i = 0; i < error_vector.rows(); ++i)
    // {
    //     onehot_temp.setZero();
    //     onehot_temp(i) = 1;
    //     double loss_element = 0.5 * error_vector(i) * (P_sparse * onehot_temp)(i)*error_vector(i);
    //     loss_sum += loss_element;
    //     if (i < MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1))
    //     {
    //         std::cout << i + 1 << ", " << i % MPC_STATE_DIM + 1 << ": " << loss_element << std::endl;
    //     }
    //     else
    //     {
    //         std::cout << i + 1 << ", " << (i - MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1)) % NUM_DOF + 1 << ": " << loss_element << std::endl;
    //     }
    // }
    // std::cout << "loss_sum: " << loss_sum << std::endl;
}

void MPCController::print_grf_plan(bool in_robot_frame)
{
    std::cout << "==== grf in world frame ==== " <<std::endl;
    for(int t = 0; t < MPC_PLAN_HORIZON; ++t)
    {
        Eigen::Matrix<double, 3, NUM_LEG> control;
        control.setZero();
        for (int i = 0; i < NUM_LEG; ++i)
        {
            if (!isnan(solution.segment<3>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + i * 3).norm()))
            {
                control.block<3, 1>(0, i) = solution.segment<3>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + i * 3 + NUM_DOF * t);
            }
        }
        std::cout << "- grf in world frame at time " << t <<std::endl;
        std::cout << control <<std::endl;
    }

}

void MPCController::debug(DogState &state, double dt)
{
    // debug_cout_trajectory(0);
    // debug_cout_trajectory(1);
    // debug_cout_trajectory(2);
    // debug_cout_trajectory(5);
    // state.attitude.debug_value[0] = state.attitude.com_euler(0);
    // state.attitude.debug_value[1] = state.attitude.com_ang_vel(0);
    // state.attitude.debug_value[2] = state.ctrl.com_euler_set(0);
    // state.attitude.debug_value[3] = state.ctrl.com_ang_vel_set(0);
    // std::cout << "pos_err" << std::endl;
    // std::cout << (state.ctrl.com_set - state.attitude.com_pos) << std::endl;
    //print_grf_plan(true);
    // debug_loss_analyse();
    state.ctrl.mpc_reference = get_reference_trajactory();
    state.ctrl.mpc_prediction = get_prediction_trajactory();
    if(is_save_log && gait_ptr_->moving_plan)
    {
        save_log(state);
    }
}

void MPCController::save_log(DogState &state)
{
    // |z_predict_pos|z_predict_vel|z_predict_force|contact|
    static int i;
    if(++i % 2 != 0)
    {
        return;
    }
    Eigen::Matrix<double, (MPC_PLAN_HORIZON + 1) * 2 + MPC_PLAN_HORIZON, 1> z_state;
    for(int i = 0; i < (MPC_PLAN_HORIZON + 1); ++i)
    {
        z_state[i] = predicted_trajectory(i * MPC_STATE_DIM + 5);
        z_state[i + MPC_PLAN_HORIZON + 1] = predicted_trajectory(i * MPC_STATE_DIM + 11);
    }

    
    for(int i = 0; i < (MPC_PLAN_HORIZON); ++i)
    {
        float grf_all = 0;
        for(int j = 0; j < NUM_LEG; ++j)
        {
            grf_all += solution[MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + i * NUM_DOF + 2 + j * 3];   
        }
        z_state[i + (MPC_PLAN_HORIZON + 1) * 2] = grf_all;
    }

    for(int i = 0; i < (MPC_PLAN_HORIZON + 1) * 2 + MPC_PLAN_HORIZON; i++)
    {
        log_file << z_state[i] << ","; 
    }
    log_file << state.attitude.com_pos(2) << "," << state.attitude.com_lin_vel_estimated(2) << "," << z_state[(MPC_PLAN_HORIZON + 1) * 2] << "," << (int)state.contact(0);
    log_file << std::endl;

}



geometry_msgs::PoseArray MPCController::get_reference_trajactory()
{
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = ros::Time::now();
    for(int i = 0; i < (MPC_PLAN_HORIZON + 1); ++i)
    {
        geometry_msgs::Pose pose;
        pose.position.x = reference(i * MPC_STATE_DIM + 3);
        pose.position.y = reference(i * MPC_STATE_DIM + 4);
        pose.position.z = reference(i * MPC_STATE_DIM + 5);
        Eigen::Quaterniond quat = Utils::euler_to_quat(reference.segment<3>(i * MPC_STATE_DIM + 0));
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        pose_array.poses.emplace_back(pose);
    }
    return pose_array;
}

geometry_msgs::PoseArray MPCController::get_prediction_trajactory()
{
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = ros::Time::now();
    for(int i = 0; i < (MPC_PLAN_HORIZON + 1); ++i)
    {
        geometry_msgs::Pose pose;
        pose.position.x = predicted_trajectory(i * MPC_STATE_DIM + 3);
        pose.position.y = predicted_trajectory(i * MPC_STATE_DIM + 4);
        pose.position.z = predicted_trajectory(i * MPC_STATE_DIM + 5);
        Eigen::Quaterniond quat = Utils::euler_to_quat(predicted_trajectory.segment<3>(i * MPC_STATE_DIM + 0));
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        pose_array.poses.emplace_back(pose);
    }
    return pose_array;
}

