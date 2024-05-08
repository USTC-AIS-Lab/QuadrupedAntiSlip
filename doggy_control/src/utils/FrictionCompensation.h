#ifndef _FRICTION_COMPENSATION_H_
#define _FRICTION_COMPENSATION_H_

#include <iostream>
#include <Eigen/Dense>

class FrictionCompensation
{
public:
    FrictionCompensation();
    FrictionCompensation(Eigen::VectorXd f_coulomb, Eigen::VectorXd beta_viscous, Eigen::VectorXd dq_threshold);
    Eigen::VectorXd compute_friction_compensation(Eigen::VectorXd dq);
private:
    Eigen::VectorXd f_coulomb_;
    Eigen::VectorXd beta_viscous_;
    Eigen::VectorXd dq_threshold_;

};


#endif 