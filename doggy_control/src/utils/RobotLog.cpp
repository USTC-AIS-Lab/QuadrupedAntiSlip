#include "RobotLog.h"
#include "Utils.h"

RobotLog::RobotLog()
{
    data_header_loaded_ = false;
    tick_ = 0;
    init();
}

void RobotLog::init()
{
    log_file_ = "/root/A1_ctrl_ws/src/dog_ctrl/doggy_control/logs/" + Utils::getCurrentTimeAsString() + ".txt";
    logger_ptr_ = spdlog::basic_logger_mt("file_logger", log_file_);
    this->info("spd_logger initialized.");
}

void RobotLog::load_log_data(Eigen::MatrixXd data, std::string name)
{
    int num_of_val = 0;
    std::string data_str = Utils::getStringFromEigen(data, &num_of_val);
    log_str_ = log_str_ + data_str + ",";
    if (!data_header_loaded_)
    {
        for (int i = 0; i < num_of_val; ++i)
        {
            data_header_.emplace_back(name + "_" + std::to_string(i + 1));
        }
    }
}


void RobotLog::load_log_data(double data, std::string name)
{
    std::string data_str = std::to_string(data);
    log_str_ = log_str_ + data_str + ",";
    if (!data_header_loaded_)
    {
        data_header_.emplace_back(name);
    }
}


void RobotLog::write_log(DogState &state)
{
    
    if(tick_ % LOG_T == 0)
    {
        log_str_.clear();
        data_header_.clear();
        load_log_data(state.attitude.com_pos, "com_pos");
        load_log_data(state.attitude.com_euler, "com_euler");
        load_log_data(state.attitude.com_ang_vel, "com_ang_vel");
        load_log_data(state.attitude.com_lin_vel, "com_lin_vel");
        
    }
    else if (tick_ % LOG_T == 1)
    {
        load_log_data(state.attitude.get_contact_state_vec() * 1.5, "is_contact");
        load_log_data(Eigen::Vector4d(state.contact(0),state.contact(1),state.contact(2),state.contact(3)), "contact");
        load_log_data(state.attitude.foot_pos, "foot_pos");
        load_log_data(state.attitude.foot_pos_r, "foot_pos_r");
        load_log_data(state.attitude.foot_vel, "foot_vel");
        // load_log_data(state.attitude.joint_pos_q, "joint_pos_q");
        // load_log_data(state.attitude.joint_vel_q, "joint_vel_q");
    }
    else if (tick_ % LOG_T == 2)
    {
        load_log_data(state.ctrl.com_set, "com_set");
        load_log_data(state.ctrl.com_euler_set, "com_euler_set");
        load_log_data(state.ctrl.com_lin_vel_set, "com_lin_vel_set");
        load_log_data(state.ctrl.foot_pos_r_set, "foot_pos_r_set");
    }
    else if (tick_ % LOG_T == 3)
    {
        // load_log_data(state.ctrl.foot_forces_grf_set, "foot_forces_grf_set");
        // load_log_data(state.ctrl.joint_torques_set, "joint_torques_set");
        // load_log_data(state.ctrl.stand_status, "stand_status");
        // load_log_data(state.ctrl.foot_forces_kin_set, "foot_forces_kin_set");
        load_log_data(double(state.ctrl.udp_online) * 10, "udp_online");
        load_log_data(state.slip_info.slip_state.cast<double>(), "slip_state");
    }
    else if(tick_ % LOG_T == 4)
    {
        load_log_data(state.attitude.debug_value[0], "debug_value_0");
        load_log_data(state.attitude.debug_value[1], "debug_value_1");
        load_log_data(state.attitude.debug_value[2], "debug_value_2");
        load_log_data(state.attitude.debug_value[3], "debug_value_3");
        load_log_data(state.attitude.debug_value[4], "debug_value_4");
        if (!data_header_loaded_)
        {
            write_log_header();
            data_header_loaded_ = true;
        }
        if(!log_str_.empty())
        {
            log_str_.pop_back(); // remove the comma at the end
        }
        logger_ptr_->info("data_log: {}", log_str_);
    }
    tick_ ++;
    
}

void RobotLog::write_log_header()
{
    std::string data_header;
    for (int i = 0; i < data_header_.size(); ++i)
    {
        data_header = data_header + data_header_.at(i);
        if (i < data_header_.size() - 1)
        {
            data_header = data_header + ",";
        }
    }
    logger_ptr_->info("data_header: {}", data_header);
}







