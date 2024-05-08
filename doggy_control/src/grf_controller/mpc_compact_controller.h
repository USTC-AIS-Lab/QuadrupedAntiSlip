#ifndef _MPC_COMPACT_CONTROLLER_H_
#define _MPC_COMPACT_CONTROLLER_H_


#define MPC_COMPACT_STATE_DIM 13
#define MPC_COMPACT_CONSTRAINT_DIM_PER_LEG 5
#define MPC_COMPACT_CONSTRAINT_DIM_u (MPC_COMPACT_CONSTRAINT_DIM_PER_LEG * NUM_LEG)
#define MPC_COMPACT_PLAN_HORIZON 10

#include "base_controller.h"
#include "gait/base_gait.h"
#include "PIDController.h"


typedef std::unique_ptr<BaseGait> GaitPtr;

//state: [0 1 2]: rpy euler angle, [3 4 5]: xyz pos, [6 7 9]: ang vel, [10 11 12]: lin vel, [13]: gravity
class MPCCompactController : public BaseController
{
public:
    MPCCompactController(GaitPtr &gait_ptr);
    Eigen::Matrix<double, 3, NUM_LEG> compute_ground_reaction_force(DogState &state, double dt);
    geometry_msgs::PoseArray get_reference_trajactory();
    geometry_msgs::PoseArray get_prediction_trajactory();

private:  
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> update_A_step(DogState &state, int time_step);
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> update_B_step(DogState &state, int time_step);
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> discretize_A(Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A);
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> discretize_B(Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, NUM_DOF> B, Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, MPC_COMPACT_STATE_DIM> A);
    void update_x_0(DogState &state);
    void update_Ac(DogState &state, double dt);
    void update_hessian(DogState &state, double dt);
    void update_bound(DogState &state);
    void update_q(DogState &state);
    void update_reference_trajectory(DogState &state, double mpc_dt);
    void update_AB_aug(DogState &state, double mpc_dt);
    void QP_solve();
    void update_prediction_trajactory();
    Eigen::Matrix<double, 3, NUM_LEG> get_control(DogState &state);

    GaitPtr &gait_ptr_;
    PIDController pid_lin;
    PIDController pid_ang;
    OsqpEigen::Solver solver;
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, 1> x_0;
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM, 1> q_weights_mpc;
    Eigen::Matrix<double, NUM_DOF, 1> r_weights_mpc;
    Eigen::DiagonalMatrix<double, MPC_COMPACT_STATE_DIM * MPC_COMPACT_PLAN_HORIZON> Q_aug;
    Eigen::DiagonalMatrix<double, NUM_DOF * MPC_COMPACT_PLAN_HORIZON> R_aug;
    Eigen::Matrix<double, NUM_DOF * MPC_COMPACT_PLAN_HORIZON, NUM_DOF * MPC_COMPACT_PLAN_HORIZON> hessian;
    Eigen::Matrix<double, MPC_COMPACT_CONSTRAINT_DIM_u * MPC_COMPACT_PLAN_HORIZON, NUM_DOF * MPC_COMPACT_PLAN_HORIZON> Ac;
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM * MPC_COMPACT_PLAN_HORIZON, MPC_COMPACT_STATE_DIM> A_aug;
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM * MPC_COMPACT_PLAN_HORIZON, NUM_DOF * MPC_COMPACT_PLAN_HORIZON> B_aug;
    Eigen::Matrix<double, NUM_DOF * MPC_COMPACT_PLAN_HORIZON, 1> q; // q
    Eigen::Matrix<double, MPC_COMPACT_CONSTRAINT_DIM_u * MPC_COMPACT_PLAN_HORIZON, 1> lb;
    Eigen::Matrix<double, MPC_COMPACT_CONSTRAINT_DIM_u * MPC_COMPACT_PLAN_HORIZON, 1> ub;
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM * (MPC_COMPACT_PLAN_HORIZON), 1> reference;
    Eigen::Matrix<double, MPC_COMPACT_STATE_DIM * (MPC_COMPACT_PLAN_HORIZON), 1> predicted_trajectory;

    Eigen::Matrix<double, NUM_DOF * MPC_COMPACT_PLAN_HORIZON, 1> solution;

    double prediction_dt;
    double F_max;
    double F_min;
    double friction_mu;
    double max_pos_error;
    double k_q_n;


};


#endif
