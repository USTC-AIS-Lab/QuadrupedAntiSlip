#include "qp_controller.h"

QPController::QPController()
{
    double kp_linear_x, kp_linear_y, kp_linear_z;
    double kd_linear_x, kd_linear_y, kd_linear_z;
    double kp_angular_x, kp_angular_y, kp_angular_z;
    double kd_angular_x, kd_angular_y, kd_angular_z;
    double qp_alpha, qp_beta;
    double qp_s_pos_x, qp_s_pos_y, qp_s_pos_z;
    double qp_s_ang_x, qp_s_ang_y, qp_s_ang_z;
    double constraint_f_max, constraint_f_min;
    double mu;
    ros::NodeHandle _nh;
    _nh.param("qp_controller/kp_linear_x", kp_linear_x, KP_LINEAR_X);
    _nh.param("qp_controller/kp_linear_y", kp_linear_y, KP_LINEAR_Y);
    _nh.param("qp_controller/kp_linear_z", kp_linear_z, KP_LINEAR_Z);
    _nh.param("qp_controller/kd_linear_x", kd_linear_x, KD_LINEAR_X);
    _nh.param("qp_controller/kd_linear_y", kd_linear_y, KD_LINEAR_Y);
    _nh.param("qp_controller/kd_linear_z", kd_linear_z, KD_LINEAR_Z);
    _nh.param("qp_controller/kp_angular_x", kp_angular_x, KP_ANGULAR_X);
    _nh.param("qp_controller/kp_angular_y", kp_angular_y, KP_ANGULAR_Y);
    _nh.param("qp_controller/kp_angular_z", kp_angular_z, KP_ANGULAR_Z);
    _nh.param("qp_controller/kd_angular_x", kd_angular_x, KD_ANGULAR_X);
    _nh.param("qp_controller/kd_angular_y", kd_angular_y, KD_ANGULAR_Y);
    _nh.param("qp_controller/kd_angular_z", kd_angular_z, KD_ANGULAR_Z);
    _nh.param("qp_controller/qp_alpha", qp_alpha, QP_ALPHA);
    _nh.param("qp_controller/qp_beta", qp_beta, QP_BETA);
    _nh.param("qp_controller/qp_s_pos_x", qp_s_pos_x, QP_S_POS_X);
    _nh.param("qp_controller/qp_s_pos_y", qp_s_pos_y, QP_S_POS_Y);
    _nh.param("qp_controller/qp_s_pos_z", qp_s_pos_z, QP_S_POS_Z);
    _nh.param("qp_controller/qp_s_ang_x", qp_s_ang_x, QP_S_ANG_X);
    _nh.param("qp_controller/qp_s_ang_y", qp_s_ang_y, QP_S_ANG_Y);
    _nh.param("qp_controller/qp_s_ang_z", qp_s_ang_z, QP_S_ANG_Z);
    _nh.param("qp_controller/constraint_f_max", constraint_f_max, CONSTRAINT_F_MAX);
    _nh.param("qp_controller/constraint_f_min", constraint_f_min, CONSTRAINT_F_MIN);
    _nh.param("friction_mu", mu, FRITION_MU);
    _nh.param("qp_controller/kp_angular_d", kp_angular_d, 3.0);
    _nh.param("qp_controller/kp_linear_d", kp_linear_d, 3.0);

    kp_linear << kp_linear_x, kp_linear_y, kp_linear_z;
    kd_linear << kd_linear_x, kd_linear_y, kd_linear_z;
    kp_angular << kp_angular_x, kp_angular_y, kp_angular_z;
    kd_angular << kd_angular_x, kd_angular_y, kd_angular_z;

    S.diagonal() << qp_s_pos_x, qp_s_pos_y, qp_s_pos_z, qp_s_ang_x, qp_s_ang_y, qp_s_ang_z;
    alpha = qp_alpha;
    beta = qp_beta;
    F_max = constraint_f_max;
    F_min = constraint_f_min;
    friction_mu = mu;
    foot_forces_grf.setZero();
    C.setZero();
    lower_bound.setZero();
    upper_bound.setZero();
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        // extract F_zi
        C(i, 2 + i * 3) = 1;
        // friction pyramid
        // 1. F_xi < uF_zi
        C(NUM_LEG + i * 4, i * 3) = 1;
        C(NUM_LEG + i * 4, 2 + i * 3) = -friction_mu;
        lower_bound(NUM_LEG + i * 4) = -OsqpEigen::INFTY;
        upper_bound(NUM_LEG + i * 4) = 0;
        // 2. F_xi > -uF_zi    => -F_xi -uF_zi < 0
        C(NUM_LEG + i * 4 + 1, i * 3) = -1;
        C(NUM_LEG + i * 4 + 1, 2 + i * 3) = -friction_mu;
        lower_bound(NUM_LEG + i * 4 + 1) = -OsqpEigen::INFTY;
        upper_bound(NUM_LEG + i * 4 + 1) = 0;
        // 3. F_yi < uF_zi
        C(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
        C(NUM_LEG + i * 4 + 2, 2 + i * 3) = -friction_mu;
        lower_bound(NUM_LEG + i * 4 + 2) = -OsqpEigen::INFTY;
        upper_bound(NUM_LEG + i * 4 + 2) = 0;
        // 4. F_yi > -uF_zi
        C(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
        C(NUM_LEG + i * 4 + 3, 2 + i * 3) = -friction_mu;
        lower_bound(NUM_LEG + i * 4 + 3) = -OsqpEigen::INFTY;
        upper_bound(NUM_LEG + i * 4 + 3) = 0;
    }
    linearMatrix = C.sparseView();
    //std::cout<< "C: " << std::endl;
    //std::cout<< C << std::endl;
    /* C:
    0    0    1    0    0    0    0    0    0    0    0    0
    0    0    0    0    0    1    0    0    0    0    0    0
    0    0    0    0    0    0    0    0    1    0    0    0
    0    0    0    0    0    0    0    0    0    0    0    1
    1    0 -0.7    0    0    0    0    0    0    0    0    0
   -1    0 -0.7    0    0    0    0    0    0    0    0    0
    0    1 -0.7    0    0    0    0    0    0    0    0    0
    0   -1 -0.7    0    0    0    0    0    0    0    0    0
    0    0    0    1    0 -0.7    0    0    0    0    0    0
    0    0    0   -1    0 -0.7    0    0    0    0    0    0
    0    0    0    0    1 -0.7    0    0    0    0    0    0
    0    0    0    0   -1 -0.7    0    0    0    0    0    0
    0    0    0    0    0    0    1    0 -0.7    0    0    0
    0    0    0    0    0    0   -1    0 -0.7    0    0    0
    0    0    0    0    0    0    0    1 -0.7    0    0    0
    0    0    0    0    0    0    0   -1 -0.7    0    0    0
    0    0    0    0    0    0    0    0    0    1    0 -0.7
    0    0    0    0    0    0    0    0    0   -1    0 -0.7
    0    0    0    0    0    0    0    0    0    0    1 -0.7
    0    0    0    0    0    0    0    0    0    0   -1 -0.7
    */
}

Eigen::Matrix<double, 3, NUM_LEG> QPController::compute_ground_reaction_force(DogState &state, double dt)
{
    Eigen::Vector3d euler_error = state.ctrl.com_euler_set - state.attitude.com_euler;
    euler_error(2) = Utils::periodical_clamp(euler_error(2), -PI, PI, 2 * PI); // clamp yaw
    state.ctrl.com_lin_vel_actuated_set = state.ctrl.com_lin_vel_set + (state.ctrl.com_pos_set - state.attitude.com_pos) * kp_linear_d;
    state.ctrl.com_ang_vel_actuated_set = state.ctrl.com_ang_vel_set + state.attitude.euler_to_ang_vel_mapping * euler_error * kp_angular_d;
    // state.ctrl.com_ang_vel_r_set(2) = 0; 
    com_dynamics_set.setZero();
    com_dynamics_set.block<3, 1>(0, 0) = kp_linear.cwiseProduct(state.ctrl.com_pos_set - state.attitude.com_pos);
    com_dynamics_set.block<3, 1>(0, 0) += kd_linear.cwiseProduct(state.ctrl.com_lin_vel_actuated_set - state.attitude.com_lin_vel);
    com_dynamics_set.block<3, 1>(3, 0) = state.attitude.euler_to_ang_vel_mapping * kp_angular.cwiseProduct(euler_error);
    com_dynamics_set.block<3, 1>(3, 0) += kd_angular.cwiseProduct(state.ctrl.com_ang_vel_actuated_set - state.attitude.com_ang_vel);
    com_dynamics_set(2) += GRAVITY_ACCELERATION;
    com_dynamics_set.block<3, 1>(0, 0) = state.attitude.robot_mass * com_dynamics_set.block<3, 1>(0, 0);
    com_dynamics_set.block<3, 1>(3, 0) = state.attitude.com_rot_mat * state.attitude.trunk_inertia * state.attitude.com_rot_mat.transpose() * com_dynamics_set.block<3, 1>(3, 0);

    // std::cout << "===========================" << std::endl;
    // std::cout << "state.ctrl.com_euler_set:" << std::endl;
    // std::cout << state.ctrl.com_euler_set << std::endl;
    // std::cout << "state.attitude.com_euler:" << std::endl;
    // std::cout << state.attitude.com_euler << std::endl;
    // std::cout << "euler_error:" << std::endl;
    // std::cout << euler_error << std::endl;
    // std::cout << "state.ctrl.com_ang_vel_actuated_set:" << std::endl;
    // std::cout << state.ctrl.com_ang_vel_actuated_set << std::endl;
    // std::cout << "state.attitude.com_ang_vel:" << std::endl;
    // std::cout << state.attitude.com_ang_vel << std::endl;
    // std::cout << "state.ctrl.com_ang_vel_actuated_set - state.attitude.com_ang_vel:" << std::endl;
    // std::cout << state.ctrl.com_ang_vel_actuated_set - state.attitude.com_ang_vel << std::endl;
    // std::cout << "com_dynamics_set: " << std::endl;
    // std::cout << com_dynamics_set << std::endl;

    


    Eigen::Matrix<double, 6, DIM_GRF> A; //matirx A in the original paper
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        Eigen::Vector3d foot_pos_abs = state.attitude.com_rot_mat * state.attitude.foot_pos_r.block<3, 1>(0, i);
        A.block<3, 3>(0, i * 3).setIdentity();
        A.block<3, 3>(3, i * 3) = Utils::skew(foot_pos_abs);
    }
    Eigen::Matrix<double, DIM_GRF, DIM_GRF> dense_hessian; // for OSQP solver
    dense_hessian = 2.0 * (A.transpose() * S * A + (alpha + beta) * Eigen::Matrix<double, DIM_GRF, DIM_GRF>::Identity());
    // std::cout << "dense_hessian: " << std::endl;
    // std::cout << dense_hessian << std::endl;
    Eigen::SparseMatrix<double> hessian = dense_hessian.sparseView();
    Eigen::Matrix<double, DIM_GRF, 1> foot_forces_grf_last;
    for(int i = 0; i < NUM_LEG; ++i)
    {
        foot_forces_grf_last.block<3, 1>(3 * i, 0) = foot_forces_grf.block<3, 1>(0, i);
    }
    Eigen::Matrix<double, DIM_GRF, 1>  gradient = 2.0 * (-A.transpose() * S * com_dynamics_set - beta * foot_forces_grf_last);
    // std::cout << "gradient: " << std::endl;
    // std::cout << gradient << std::endl;
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        double c_flag = state.contact(i) ? 1.0 : 0.0;
        lower_bound(i) = c_flag * F_min;
        upper_bound(i) = c_flag * F_max;
    }
    // std::cout << "lower_bound: " << std::endl;
    // std::cout << lower_bound << std::endl;
    // std::cout << "upper_bound: " << std::endl;
    // std::cout << upper_bound << std::endl;
    OsqpEigen::Solver solver;
    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(3 * NUM_LEG);
    solver.data()->setNumberOfConstraints(NUM_LEG + 4 * NUM_LEG);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);
    auto t1 = std::chrono::high_resolution_clock::now();
    solver.initSolver();
    auto t2 = std::chrono::high_resolution_clock::now();
    solver.solveProblem();
    auto t3 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
    std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;

    //std::cout << "qp solver init time: " << ms_double_1.count() << "ms; solve time: " << ms_double_2.count() << "ms" << std::endl;

    Eigen::VectorXd QPSolution = solver.getSolution(); //12x1
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        //convert the force into robot frame
        foot_forces_grf.block<3, 1>(0, i) = state.attitude.com_rot_mat.transpose() * QPSolution.segment<3>(i * 3);
    }
    // std::cout << "foot_forces_grf: " << std::endl;
    // std::cout << foot_forces_grf << std::endl;
    // std::cout << "com_pos: " << std::endl;
    // std::cout << state.attitude.com_pos << std::endl;
    // std::cout << "com_dynamics_set: " << std::endl;
    // std::cout << com_dynamics_set << std::endl;
    // std::cout << "actuated dynamics: " << std::endl;
    // std::cout << A * QPSolution << std::endl;
    // std::cout << "euler_error: " << std::endl;
    // std::cout << euler_error << std::endl;
    // std::cout << "pos_error: " << std::endl;
    // std::cout << state.ctrl.com_pos_set - state.attitude.com_pos << std::endl;
    // std::cout << "contact: " << std::endl;
    // std::cout << state.contact(0) <<", "<< state.contact(1) <<", "<< state.contact(2) <<", "<< state.contact(3) << std::endl;




    //foot_forces_grf.setZero();




    //state.ctrl.foot_forces_grf_set = foot_forces_grf;
    return foot_forces_grf;
    // state.attitude.debug_value[0] = euler_error(2);
    // state.attitude.debug_value[1] = com_ang_vel_set(2);

}

