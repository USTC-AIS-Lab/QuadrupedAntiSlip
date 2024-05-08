#ifndef REAL_DOG_H
#define REAL_DOG_H

#include <iostream>
#include <Eigen/Dense>
#include <thread>
#include <algorithm>

#include <ros/ros.h>


#include "dog_params.h"
#include "dog_state.h"
#include "dog_control.h"
#include "StateEstimate.h"
#include "InfoToROS.h"
#include "a1_kinematics.h"
#include "Utils.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "utils/vio_udp.h"
#include "FrictionCompensation.h"

// FL, FR, RL, RR
extern const int joint_remap[12];
extern const int foot_remap[4];

class UDPLowComm
{
public:
    UDPLowComm(int _power_level) : safe(UNITREE_LEGGED_SDK::LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL), power_level(_power_level)
    {
        udp.InitCmdData(cmd);
    }
    void receive()
    {
        udp.Recv();
        udp.GetRecv(state);
    }
    void send_init();
    void send()
    {
        safe.PositionLimit(cmd);
        udp.SetSend(cmd);
        safe.PowerProtect(cmd, state, power_level);
        udp.Send();
    }
    int power_level;
    UNITREE_LEGGED_SDK::UDP udp;
    UNITREE_LEGGED_SDK::Safety safe;
    UNITREE_LEGGED_SDK::LowState state = {0};
    UNITREE_LEGGED_SDK::LowCmd cmd = {0};
};

class RealDog
{
public:
    RealDog(ros::NodeHandle &nh);
    ~RealDog()
    {
        thread_distructed_ = true; // stop udp thread
        receve_udp_thread_.join();
        send_ros_info_thread_.join();
        write_log_thread_.join();
    }
    bool send_cmd();
    bool main_update(double dt);
    void wait_for_udp_received();
    bool update_foot_forces_grf(double dt);
    bool safety_halt(){return state_.ctrl.safety_halt;}
    void doggy_cmd_callback(const doggy_msgs::DoggyMove &msg);
    template <typename T>
    void log(spdlog::level::level_enum lvl, const T &msg);
    template <typename... Args>
    void log(spdlog::level::level_enum lvl, spdlog::format_string_t<Args...> fmt, Args &&...args);

private:
    ros::NodeHandle nh_;
    DogState state_;
    DogControl dog_control_;
    StateEstimate state_estimate_;
    ros::Subscriber doggy_cmd_sub_;
    UDPLowComm udp_comm_;
    InfoToROS info_to_ros_;
    std::thread receve_udp_thread_;
    std::thread send_ros_info_thread_;
    std::thread write_log_thread_;
    std::unique_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_vio_ptr_;
    VIO_UDP vio_udp_;
    FrictionCompensation friction_compensation_;
    bool thread_distructed_;
    bool udp_received_;
    void receive_udp_state();
    void receive_vio_data();
    void send_ros_info();
    void write_log();
};

#endif
