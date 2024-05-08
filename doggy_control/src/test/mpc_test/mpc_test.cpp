#include "mpc_test.h"


MPCTest::MPCTest()
{
    prediction_dt = 0.5;
    max_u = 5;
    x_offset = 1000;
    x_0.setZero();
    for(int i = 0; i < MPC_STATE_DIM; ++i)
    {
        if(i < MPC_STATE_DIM / 2)
        {
            q_weights_mpc(i) = 0.5;
        }
        else
        {
            q_weights_mpc(i) = 0.01;
        }
        
    }
    for(int i = 0; i < MPC_CTRL_DIM; ++i)
    {
        r_weights_mpc(i) = 1e-4;
    }
    target_point << 5 + x_offset,5,5,0,0,0;
    
    Q_mpc.diagonal() = q_weights_mpc;
    R_mpc.diagonal() = r_weights_mpc;

    Ac_sparse = Eigen::SparseMatrix<double>((MPC_STATE_DIM) * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON);

    P_sparse = Eigen::SparseMatrix<double>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON);

    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON> P_dense;
    P_dense.setZero();
    for(int i = 0; i < MPC_PLAN_HORIZON + 1; ++i)
    {
        P_dense.block<MPC_STATE_DIM,MPC_STATE_DIM>(MPC_STATE_DIM * i, MPC_STATE_DIM * i) = Q_mpc;
    }
    for(int i = 0; i < MPC_PLAN_HORIZON; ++i)
    {
        P_dense.block<MPC_CTRL_DIM,MPC_CTRL_DIM>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * i, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * i) = R_mpc;
    }
    P_sparse = P_dense.sparseView();
    // std::cout << "P_sparse: "<< std::endl;
    // std::cout << P_sparse << std::endl;


    A.setZero();
    A.block<MPC_STATE_DIM/2, MPC_STATE_DIM/2>(0, MPC_STATE_DIM/2) = Eigen::Matrix<double, MPC_STATE_DIM/2, MPC_STATE_DIM/2>::Identity();
    B.setZero();
    B.block<MPC_STATE_DIM/2, MPC_STATE_DIM/2>(MPC_STATE_DIM/2, 0) = Eigen::Matrix<double, MPC_STATE_DIM/2, MPC_STATE_DIM/2>::Identity();
    // std::cout << "A: "<< std::endl;
    // std::cout << A << std::endl;

    // std::cout << "B: "<< std::endl;
    // std::cout << B << std::endl;

}

void MPCTest::update_x_0()
{
    for(int i = 0; i < MPC_STATE_DIM / 2; ++i)
    {
        x_0(i) = 0.2;
    }
    x_0(0) += x_offset;
}

void MPCTest::update_reference_trajectory()
{
    reference.setZero();
    reference.segment<MPC_STATE_DIM>(0) = x_0;
    for (int time_step = 0; time_step < MPC_PLAN_HORIZON; time_step++)
    {
        reference.segment<MPC_STATE_DIM>((time_step + 1) * MPC_STATE_DIM) = target_point;
    }
    // std::cout << "reference: " << std::endl;
    // std::cout << reference << std::endl;
}

void MPCTest::update_Ac()
{
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_d;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_CTRL_DIM> B_d;
    Eigen::Matrix<double, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON> Ac_dense;
    Ac_dense.setZero();
    // A_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A * prediction_dt + 0.5 * prediction_dt * prediction_dt * A * A;
    // B_d = (prediction_dt * Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity()  + 0.5 * A * prediction_dt * prediction_dt) * B;
    A_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A * prediction_dt;
    B_d = prediction_dt * Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() * B;
    for(int i = 0; i < MPC_PLAN_HORIZON + 1; ++i)
    {
        Ac_dense.block<MPC_STATE_DIM, MPC_STATE_DIM>(i * MPC_STATE_DIM, i * MPC_STATE_DIM) = - Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity();
    }
    for(int i = 0; i < MPC_PLAN_HORIZON; ++i)
    {
        Ac_dense.block<MPC_STATE_DIM, MPC_STATE_DIM>(i * MPC_STATE_DIM + MPC_STATE_DIM, i * MPC_STATE_DIM ) = A_d;
        Ac_dense.block<MPC_STATE_DIM, MPC_CTRL_DIM>(i * MPC_STATE_DIM + MPC_STATE_DIM, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + i * MPC_CTRL_DIM ) = B_d;
        Ac_dense.block<MPC_CONSTRAINT_DIM_u, MPC_CTRL_DIM>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x) * MPC_PLAN_HORIZON + i * MPC_CONSTRAINT_DIM_u, MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + i * MPC_CONSTRAINT_DIM_u) = Eigen::Matrix<double, MPC_CONSTRAINT_DIM_u, MPC_CTRL_DIM>::Identity();
    }
    Ac_sparse = Ac_dense.sparseView();

    // std::cout << "Ac_sparse: " << std::endl;
    // std::cout << Ac_sparse << std::endl;
}

void MPCTest::update_bound()
{
    lb.setZero();
    ub.setZero();
    lb.segment<MPC_STATE_DIM>(0) = -x_0;
    ub.segment<MPC_STATE_DIM>(0) = -x_0;
    for (int time_step = 0; time_step < MPC_PLAN_HORIZON; time_step++)
    {
        for(int i = 0; i < MPC_CONSTRAINT_DIM_u; ++i)
        {
            lb(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x) * MPC_PLAN_HORIZON + time_step * MPC_CONSTRAINT_DIM_u + i) = - max_u;
            ub(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x) * MPC_PLAN_HORIZON + time_step * MPC_CONSTRAINT_DIM_u + i) = max_u;
        }
    }
    // std::cout << "lb: " << std::endl;
    // std::cout << lb << std::endl;
    // std::cout << "ub: " << std::endl;
    // std::cout << ub << std::endl;

}

void MPCTest::update_q()
{
    q.setZero();
    Eigen::Matrix<double, MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON, 1> reference_no_input;
    reference_no_input.setZero();
    reference_no_input.segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0) = reference;
    q.segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0) = -(P_sparse * reference_no_input).segment<MPC_STATE_DIM *(MPC_PLAN_HORIZON + 1)>(0);
    // std::cout << "q: " << std::endl;
    // std::cout << q << std::endl;
}

void MPCTest::QP_solve()
{
    if (!solver.isInitialized())
    {
        
        solver.settings()->setVerbosity(true); 
        solver.settings()->setWarmStart(true);
        solver.settings()->setRelativeTolerance(1e-4);
        solver.settings()->setAbsoluteTolerance(1e-4);
        solver.settings()->setLinearSystemSolver(QDLDL_SOLVER); // QDLDL_SOLVER MKL_PARDISO_SOLVER
        solver.data()->setNumberOfVariables(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1) + MPC_CTRL_DIM * MPC_PLAN_HORIZON);
        solver.data()->setNumberOfConstraints((MPC_STATE_DIM) * (MPC_PLAN_HORIZON + 1) + (MPC_CONSTRAINT_DIM_x + MPC_CONSTRAINT_DIM_u) * MPC_PLAN_HORIZON);
        solver.data()->setLinearConstraintsMatrix(Ac_sparse);
        solver.data()->setHessianMatrix(P_sparse);
        solver.data()->setGradient(q);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);
        solver.initSolver();
    }
    else
    {
        solver.updateLinearConstraintsMatrix(Ac_sparse);
        solver.updateGradient(q);
        solver.updateBounds(lb, ub);

    }
    solver.solveProblem();
    solution = solver.getSolution();
    predicted_trajectory = solution.segment<MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1)>(0);

}

void MPCTest::debug_cout_trajectory(int value_index)
{
    std::string value_name_list[3] = {"x", "y", "z"};
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
        std::cout << reference(MPC_STATE_DIM * i + debug_index + MPC_STATE_DIM / 2) << " ";
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
        std::cout << predicted_trajectory(MPC_STATE_DIM * i + debug_index + MPC_STATE_DIM / 2) << " ";
    }
    std::cout << std::endl;
}

void MPCTest::solve()
{
    update_x_0();
    update_reference_trajectory();
    update_Ac();
    update_bound();
    update_q();
    QP_solve();
    debug_cout_trajectory(0);
    // std::cout << solution << std::endl;
}


int main(int argc, char **argv)
{
    MPCTest mpc_test;
    mpc_test.solve();
    return 0;
}