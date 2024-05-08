#include "anti_slip.h"

AntiSlip::AntiSlip(GaitPtr &gait_ptr): slip_detector_(gait_ptr), gait_ptr_(gait_ptr)
{
    for(int i = 0; i < NUM_LEG; ++i)
    {
        impedance_activated[i] = false;
    }
    probe_force_.setZero();
    probing_ = false;
}

void AntiSlip::slip_detect(DogState &state)
{
    slip_detector_.detect(state);
}

Eigen::Matrix<double, 3, NUM_LEG> AntiSlip::impedance_control (DogState &state)
{
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_imp_raw = impedance_controller_.compute_impedance_force(state);
    foot_forces_imp_set_.setZero();
    for(int i = 0; i < NUM_LEG; ++i)
    {
        impedance_activated[i] = state.slip_info.slip_state(i);
    }

    for(int i = 0; i < NUM_LEG; ++i)
    {
        if(i == state.slip_info.probe_leg && probing_)
        {
            foot_forces_imp_set_.block<3, 1>(0, i) = probe_force_;
            foot_forces_imp_set_(2, i) = 0;
        }
        else if(impedance_activated[i])
        {
            foot_forces_imp_set_.block<3, 1>(0, i) = foot_forces_imp_raw.block<3, 1>(0, i);
        }
    }
    
    // std::cout << gait_ptr_->get_gait_name() << std::endl;
    return foot_forces_imp_set_;
}

Eigen::Matrix<double, NUM_DOF, 1> AntiSlip::torque_modify(Eigen::Matrix<double, NUM_DOF, 1> joint_torques_raw, DogState &state)
{
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques = joint_torques_raw;
    for(int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Matrix3d jac = state.attitude.j_foot_r.block<3, 3>(3 * i, 3 * i);
        if (state.contact(i))
        {
            joint_torques.segment<3>(i * 3) += jac.transpose() * foot_forces_imp_set_.block<3, 1>(0, i);
        }
        else
        {
            // swing leg PD
            // joint_torques.segment<3>(i * 3) = jac.transpose() * state.ctrl.foot_forces_kin_set.block<3, 1>(0, i);
            // joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt);
        }
    }
    return joint_torques;
}

void AntiSlip::friction_probe(DogState &state)
{
    static bool first_contact = true;
    static Eigen::Vector3d first_contact_spot = Eigen::Vector3d::Zero();
    probing_ = gait_ptr_->get_gait_name() == "probe_gait" && state.slip_info.is_probing;
    if(probing_)
    {
        probe_force_(2) = 25.0;
        if(state.attitude.is_contact[state.slip_info.probe_leg])
        {
            if(first_contact)
            {
                first_contact = false;
                first_contact_spot = state.attitude.foot_pos.block<3, 1>(0, state.slip_info.probe_leg);
            }
            
            probe_force_(0) += -0.01;
            probe_force_(1) += 0.01;

            double max_lateral_force = Eigen::Vector2d(probe_force_(0), probe_force_(1)).norm();
            if((state.attitude.foot_pos.block<3, 1>(0, state.slip_info.probe_leg) - first_contact_spot).norm() > 0.02)
            {
                
                state.slip_info.probe_slid_detected = true;
                
                double calculated_mu = max_lateral_force / probe_force_(2);
                std::cout << "probe finished by slippage, max lateral force: " << max_lateral_force <<", calculated mu: " << calculated_mu << std::endl;
                probe_force_.setZero();
            }
            else if(max_lateral_force > 30)
            {
                state.slip_info.probe_slid_detected = true;
                double calculated_mu = max_lateral_force / probe_force_(2);
                std::cout << "probe finished by maximum force : " << max_lateral_force <<", mu more than: " << calculated_mu << std::endl;
                probe_force_.setZero();
            }   


            // probe_force_(0) = -state.ctrl.doggy_cmd.reverve_val1 * 20 / 1.414;
            // probe_force_(1) = state.ctrl.doggy_cmd.reverve_val1 * 20 / 1.414;
        }
    }
    else
    {
        probe_force_.setZero();
        first_contact = true;
    }
    probe_force_ = probe_force_;
    state.slip_info.probe_force = probe_force_;
}
