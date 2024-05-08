#ifndef QP_CONTROLLER_H
#define QP_CONTROLLER_H

#include "base_controller.h"

class QPController : public BaseController
{
public:
    QPController();
    Eigen::Matrix<double, 3, NUM_LEG> compute_ground_reaction_force(DogState &state, double dt);
private:  
    Eigen::Vector3d kp_linear; // for ground reaction force control
    Eigen::Vector3d kd_linear;
    Eigen::Vector3d kp_angular;
    Eigen::Vector3d kd_angular;
    Eigen::Matrix<double, 6, 1> com_dynamics_set;
    Eigen::Matrix<double, NUM_LEG * 5, NUM_LEG * 3> C;
    Eigen::Matrix<double, NUM_LEG * 5, 1> d;
    Eigen::Matrix<double, NUM_LEG * 5, 1> lower_bound;
    Eigen::Matrix<double, NUM_LEG * 5, 1> upper_bound;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::DiagonalMatrix<double, 6> S;
    double alpha;
    double beta;
    double F_max;
    double F_min;
    double friction_mu;
    double kp_angular_d;
    double kp_linear_d;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
};

#endif