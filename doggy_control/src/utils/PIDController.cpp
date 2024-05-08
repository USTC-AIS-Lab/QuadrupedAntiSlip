#include "PIDController.h"
#include "Utils.h"

PIDController::PIDController()
{
    dim_ = 0;
    kp_.setZero();
    ki_.setZero();
    kd_.setZero();
    p_max_.setZero();
    i_max_.setZero();
    d_max_.setZero();
    all_max_.setZero();
    val_set_.setZero();
    val_now_.setZero();
    p_out_.setZero();
    i_out_.setZero();
    d_out_.setZero();
    all_out_.setZero();
    err_.setZero();
    err_last_.setZero();
}



void PIDController::set_param(int dim, Eigen::VectorXd kp, Eigen::VectorXd ki, Eigen::VectorXd kd, Eigen::VectorXd p_max, Eigen::VectorXd i_max, Eigen::VectorXd d_max, Eigen::VectorXd all_max)
{
    dim_ = dim;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    p_max_ = p_max;
    i_max_ = i_max;
    d_max_ = d_max;
    all_max_ = all_max;
    val_set_.resize(kp.size());
    val_now_.resize(kp.size());
    p_out_.resize(kp.size());
    i_out_.resize(kp.size());
    d_out_.resize(kp.size());
    all_out_.resize(kp.size());
    err_.resize(kp.size());
    err_last_.resize(kp.size());


}

Eigen::VectorXd PIDController::cal_output(Eigen::VectorXd val_now, Eigen::VectorXd val_set)
{
    val_now_ = val_now;
    val_set_ = val_set;
    err_ = val_set_ - val_now_;
    p_out_ = Utils::clamp(kp_.cwiseProduct(err_), -p_max_, p_max_);
    i_out_ = Utils::clamp(i_out_ + ki_.cwiseProduct(err_), -i_max_, i_max_);
    d_out_ = Utils::clamp(kd_.cwiseProduct(err_ - err_last_), -d_max_, d_max_);
    all_out_ = Utils::clamp(p_out_ + i_out_ + d_out_, -all_max_, all_max_);
    err_last_ = err_;
    return all_out_;
}