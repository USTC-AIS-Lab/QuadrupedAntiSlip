#include "SlipDetection.h"

SlipDetector::SlipDetector(GaitPtr &gait_ptr): gait_ptr_(gait_ptr)
{
    ros::NodeHandle _nh;
    _nh.param("anti_slip/detect_kd", detect_kd, 5.0);
    _nh.param("anti_slip/detect_kp", detect_kp, 5.0);
    
    _nh.param("anti_slip/lateral_vel_filter", lateral_vel_filter, 0.9);
    foot_lateral_vel_norm.setZero();
    slip_state.setZero();
    slip_score.setZero();
    slip_score_filtered.setZero();
    foot_lateral_dis_norm.setZero();
    foot_first_contact_spot.setZero();
    last_contact_state.setZero();
    
}


void SlipDetector::detect(DogState &state)
{
    for(int i = 0; i < NUM_LEG; ++i)
    {
        if(state.contact(i) > last_contact_state(i))
        {
            foot_first_contact_spot.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i);
        }
        foot_lateral_dis_norm(i) = state.contact(i) * (foot_first_contact_spot.block<3, 1>(0, i) - state.attitude.foot_pos.block<3, 1>(0, i)).segment<2>(0).norm();

        foot_lateral_vel_norm(i) = state.contact(i) * (foot_lateral_vel_norm(i) * lateral_vel_filter + state.attitude.foot_vel_filtered.block<2, 1>(0, i).norm() * (1.0 - lateral_vel_filter));
        slip_score(i) = state.contact(i) * (0.5 * (std::erf(detect_kd * foot_lateral_vel_norm(i) + detect_kp * foot_lateral_dis_norm(i) - 2.0) + 1.0));
        double filter_ratio = slip_score(i) > slip_score_filtered(i) ? 0.9 : 0.8;
        slip_score_filtered(i) = filter_ratio * slip_score_filtered(i) + (1.0 - filter_ratio) * slip_score(i);
        slip_state(i) = slip_score_filtered(i) > 0.5;
        if(slip_state(i) && !state.slip_info.slip_state(i)) // set this contact to be slippery
        {
            state.slip_info.slip_state(i) = slip_state(i);
            foot_slip_spot.block<3, 1>(0, i) = state.attitude.foot_pos.block<3, 1>(0, i);
            state.slip_info.foot_slip_spot.block<3, 1>(0, i) = foot_slip_spot.block<3, 1>(0, i);
        }
        if(gait_ptr_->is_in_swing_phase(i) && gait_ptr_->is_moving[i]) // clear the slippery state during swing phase.
        {
            state.slip_info.slip_state(i) = 0;
        }
        last_contact_state(i) = state.contact(i);
    }
    
    state.attitude.debug_value[0] = foot_lateral_vel_norm(1) ;
    state.attitude.debug_value[1] = slip_score_filtered(1) ;
    state.attitude.debug_value[2] = state.slip_info.slip_state(1) ;
    state.attitude.debug_value[3] = state.ctrl.foot_forces_imp_set.block<3, 1>(0, 1).norm();
    state.attitude.debug_value[4] = foot_lateral_dis_norm(1) ;

}
