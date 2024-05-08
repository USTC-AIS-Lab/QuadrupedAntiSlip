#ifndef _MPC_CONTROLLER_H
#define _MPC_CONTROLLER_H

#define MPC_STATE_DIM 13
#define MPC_CONSTRAINT_DIM_PER_LEG 5
#define MPC_CONSTRAINT_DIM_u (MPC_CONSTRAINT_DIM_PER_LEG * NUM_LEG)
#define MPC_CONSTRAINT_DIM_x 0
#define MPC_PLAN_HORIZON 8

#include "base_controller.h"
#include "gait/base_gait.h"
#include "PIDController.h"
#include <fstream>



typedef std::unique_ptr<BaseGait> GaitPtr;

//state: [0 1 2]: rpy euler angle, [3 4 5]: xyz pos, [6 7 9]: ang vel, [10 11 12]: lin vel, [13]: gravity
class MPCController : public BaseController
{
public:
    MPCController(GaitPtr &gait_ptr);
    Eigen::Matrix<double, 3, NUM_LEG> compute_ground_reaction_force(DogState &state, double dt);
    void debug_cout_trajectory(int value_index);
    void debug_loss_analyse();
    void debug(DogState &state, double dt);
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, 1> get_solution(){return solution;};
    geometry_msgs::PoseArray get_reference_trajactory();
    geometry_msgs::PoseArray get_prediction_trajactory();
    void print_grf_plan(bool in_robot_frame);

private:  
    GaitPtr &gait_ptr_;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> update_A_step(DogState &state, int time_step);
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> update_B_step(DogState &state, int time_step);
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> discretize_A(Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A);
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> discretize_B(Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B, Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A);
    void update_x_0(DogState &state);
    void update_Ac(DogState &state, double dt);
    void update_bound(DogState &state);
    void update_q(DogState &state);
    void update_reference_trajectory(DogState &state, double mpc_dt);
    void QP_solve();
    Eigen::Matrix<double, 3, NUM_LEG> get_control(DogState &state);
    Eigen::Matrix<double, MPC_STATE_DIM, 1> x_0;
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1), 1> reference;
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1), 1> predicted_trajectory;
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, 1> solution;
    double prediction_dt;
    double F_max;
    double F_min;
    double friction_mu;
    double max_pos_error;
    
    PIDController pid_lin;
    PIDController pid_ang;
    
    OsqpEigen::Solver solver;
    Eigen::Matrix<double, MPC_STATE_DIM, 1> q_weights_mpc;
    Eigen::Matrix<double, NUM_DOF, 1> r_weights_mpc;
    Eigen::DiagonalMatrix<double, MPC_STATE_DIM> Q_mpc;
    double k_q_n;
    Eigen::DiagonalMatrix<double, NUM_DOF> R_mpc;
    Eigen::SparseMatrix<double> P_sparse;
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + NUM_DOF * MPC_PLAN_HORIZON, 1> q; // q
    Eigen::SparseMatrix<double> Ac_sparse;
    std::vector<Eigen::Triplet<double>> triplet_list_Ac_static; // static elements in A_c, stored in the form of triplet_list
    std::vector<Eigen::Triplet<double>> triplet_list_Ac;
    Eigen::Matrix<double, (MPC_STATE_DIM) * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON, 1> lb;
    Eigen::Matrix<double, (MPC_STATE_DIM) * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON, 1> ub;

    std::ofstream log_file;
    bool is_save_log;
    void save_log(DogState &state);


};

#endif