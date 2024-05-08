#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Eigen/Dense>


class PIDController
{
public:
    PIDController();
    void set_param(int dim, Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd, Eigen::VectorXd p_max, Eigen::VectorXd i_max, Eigen::VectorXd d_max, Eigen::VectorXd all_max);
    Eigen::VectorXd cal_output(Eigen::VectorXd val_now, Eigen::VectorXd val_set);
private:
    int dim_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd ki_;
    Eigen::VectorXd kd_;
    Eigen::VectorXd p_max_;
    Eigen::VectorXd i_max_;
    Eigen::VectorXd d_max_;
    Eigen::VectorXd all_max_;
    Eigen::VectorXd val_set_;
    Eigen::VectorXd val_now_;
    Eigen::VectorXd p_out_;
    Eigen::VectorXd i_out_;
    Eigen::VectorXd d_out_;
    Eigen::VectorXd all_out_;
    Eigen::VectorXd err_;
    Eigen::VectorXd err_last_;

    

};



#endif
