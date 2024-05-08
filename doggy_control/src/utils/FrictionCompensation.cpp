#include "FrictionCompensation.h"

FrictionCompensation::FrictionCompensation()
{
    f_coulomb_ = Eigen::Matrix<double, 12, 1>::Zero();
    beta_viscous_ = Eigen::Matrix<double, 12, 1>::Zero();
    dq_threshold_ = Eigen::Matrix<double, 12, 1>::Zero();

    // for(int i = 0; i < 12; ++i)
    // {
    //     if((i + 1) % 3 == 0) // calf
    //     {
    //         f_coulomb_[i] = 0.4; 
    //         beta_viscous_[i] = 0.02;
    //         dq_threshold_[i] = 0.01;
    //     }
    //     else if((i + 1) % 3 == 1) // hip
    //     {
    //         f_coulomb_[i] = 0.4; 
    //         beta_viscous_[i] = 0.02;
    //         dq_threshold_[i] = 0.02;
    //     }
    //     else if((i + 1) % 3 == 2) // thigh
    //     {
    //         f_coulomb_[i] = 0.1; 
    //         beta_viscous_[i] = 0.02;
    //         dq_threshold_[i] = 0.1;
    //     }



    //     // f_coulomb_[i] = 0.0; 
    // }
}

FrictionCompensation::FrictionCompensation(Eigen::VectorXd f_coulomb,
                                           Eigen::VectorXd beta_viscous,
                                           Eigen::VectorXd dq_threshold)
    : f_coulomb_(f_coulomb),
      beta_viscous_(beta_viscous),
      dq_threshold_(dq_threshold)
{

}

Eigen::VectorXd FrictionCompensation::compute_friction_compensation(Eigen::VectorXd dq)
{
    Eigen::VectorXd friction_compensation = dq;
    for(int i = 0; i < dq.rows(); ++i)
    {
        if(dq[i] > dq_threshold_[i])
        {
            friction_compensation[i] = f_coulomb_[i] + beta_viscous_[i] * dq[i];
        }
        else if(dq[i] < -dq_threshold_[i])
        {
            friction_compensation[i] = - f_coulomb_[i] + beta_viscous_[i] * dq[i];
        }
        else
        {
            friction_compensation[i] = 0.0;
        }
    }
    return friction_compensation;
}

