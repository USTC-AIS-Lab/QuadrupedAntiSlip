#ifndef _MPC_TEST_H
#define _MPC_TEST_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include "Utils.h"

#define MPC_STATE_DIM 6
#define MPC_CTRL_DIM 3
#define MPC_PLAN_HORIZON 10
#define MPC_CONSTRAINT_DIM_x 0
#define MPC_CONSTRAINT_DIM_u MPC_CTRL_DIM


class MPCTest
{
public:
    MPCTest();
    void solve();
    Eigen::Matrix<double, MPC_STATE_DIM, 1> target_point;

private:
    double prediction_dt;
    double max_u;
    double x_offset;
    OsqpEigen::Solver solver;
    Eigen::Matrix<double, MPC_STATE_DIM, 1> x_0;
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1), 1> reference;
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1), 1> predicted_trajectory;
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON, 1> solution;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_CTRL_DIM> B;
    Eigen::SparseMatrix<double> Ac_sparse;
    Eigen::SparseMatrix<double> P_sparse;
    
    Eigen::Matrix<double, (MPC_STATE_DIM) * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON, 1> lb;
    Eigen::Matrix<double, (MPC_STATE_DIM) * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON, 1> ub;
    
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON, 1> q;

    Eigen::DiagonalMatrix<double, MPC_STATE_DIM> Q_mpc;
    Eigen::DiagonalMatrix<double, MPC_CTRL_DIM> R_mpc;
    Eigen::Matrix<double, MPC_STATE_DIM, 1> q_weights_mpc;
    Eigen::Matrix<double, MPC_CTRL_DIM, 1> r_weights_mpc;

    void update_x_0();
    void update_reference_trajectory();
    void update_Ac();
    void update_bound();
    void update_q();
    void debug_cout_trajectory(int value_index);
    void QP_solve();


};


#endif
