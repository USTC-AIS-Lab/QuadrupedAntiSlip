#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <mkl.h>
#include "test/osqp_mkl_test.h"

using namespace std;
using namespace Eigen;




void osqp_mkl_test()
{
    cout << "start" << endl;

    // int i = MKL_Get_Max_Threads();

    // cout << "max thread " << i << endl;

    Matrix2d hessian_dense;
    hessian_dense << 4, 1, 1, 2;
    // hessian_dense = hessian_dense * 1e-4;
    Eigen::SparseMatrix<double> hessian = hessian_dense.sparseView();
    cout << "H = " << endl
         << hessian_dense << endl
         << endl;

    Vector2d f;
    f << 1, 1;
    cout << "f = " << endl
         << f << endl
         << endl;
     // f = f *1e-4;

    Vector3d c_l;
    c_l << 1, 0, 0;
    cout << "c_l = " << endl
         << c_l << endl
         << endl;

    Vector3d c_u;
    c_u << 1, 0.7, 0.7;
    cout << "c_u = " << endl
         << c_u << endl
         << endl;

    MatrixXd A_dense(3, 2);
    A_dense << 1, 1, 1, 0, 0, 1;
    Eigen::SparseMatrix<double> A = A_dense.sparseView();
    cout << "A = " << endl
         << A_dense << endl
         << endl;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(true);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAlpha(1.0f);
    solver.settings()->setLinearSystemSolver(MKL_PARDISO_SOLVER); // MKL_PARDISO_SOLVER QDLDL_SOLVER
    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(3);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setGradient(f);
    solver.data()->setLowerBound(c_l);
    solver.data()->setUpperBound(c_u);

    solver.initSolver();
    solver.solveProblem();

    VectorXd QPSolution = solver.getSolution();
    cout << "solution = " << endl
         << QPSolution << endl
         << endl;
}
