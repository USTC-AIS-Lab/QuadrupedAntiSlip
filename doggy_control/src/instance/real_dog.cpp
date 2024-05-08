#include "real_dog.h"

// FL, FR, RL, RR
const int JOINT_REMAP[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
const int FOOT_REMAP[4] = {1, 0, 3, 2};

RealDog::RealDog(ros::NodeHandle &nh) : nh_(nh), udp_comm_(SAFE_POWER_LEVEL), info_to_ros_(nh)
{
    thread_distructed_ = false;
    udp_received_ = false;
    doggy_cmd_sub_ = nh_.subscribe("/doggy_cmd", 100, &RealDog::doggy_cmd_callback, this);
    receve_udp_thread_ = std::thread(&RealDog::receive_udp_state, this);
    send_ros_info_thread_ = std::thread(&RealDog::send_ros_info, this);
    write_log_thread_ = std::thread(&RealDog::write_log, this);
    // loop_vio_ptr_  = std::make_unique<UNITREE_LEGGED_SDK::LoopFunc>("udp_recv", vio_udp_.get_dt(), 3, boost::bind(&RealDog::receive_vio_data, this));
    // loop_vio_ptr_->start();
}

void RealDog::doggy_cmd_callback(const doggy_msgs::DoggyMove &msg)
{
    state_.ctrl.doggy_cmd = msg;
    state_.ctrl.doggy_cmd_received = true;
}

void RealDog::receive_udp_state()
{
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    int sig_loss_count = 0;
    int last_seq = 0;
    while (thread_distructed_ == false)
    {
        auto t1 = ros::Time::now();
        /*********************raw data receive********************/
        udp_comm_.receive();
        if(last_seq != udp_comm_.state.tick)
        {
            last_seq = udp_comm_.state.tick;
            sig_loss_count = 0;
            state_.ctrl.udp_online = true;
        }
        else
        {
            sig_loss_count ++;
            if(sig_loss_count >= 10)
            {
                state_.ctrl.udp_online = false;
                if((sig_loss_count - 10) % 1000 == 0)
                {
                    dog_control_.log(spdlog::level::err ,"udp offline!!");
                }
                
            }
        }
        
        state_.attitude.com_quat = Eigen::Quaterniond(udp_comm_.state.imu.quaternion[0],
                                                       udp_comm_.state.imu.quaternion[1],
                                                       udp_comm_.state.imu.quaternion[2],
                                                       udp_comm_.state.imu.quaternion[3]); 
        state_.attitude.imu_acc = Eigen::Vector3d(udp_comm_.state.imu.accelerometer[0], udp_comm_.state.imu.accelerometer[1], udp_comm_.state.imu.accelerometer[2]);
        state_.attitude.com_ang_vel_r = Eigen::Vector3d(udp_comm_.state.imu.gyroscope[0], udp_comm_.state.imu.gyroscope[1], udp_comm_.state.imu.gyroscope[2]);
        for (int i = 0; i < NUM_LEG; ++i)
        {
            state_.attitude.foot_force[i] = (double)(udp_comm_.state.footForce[FOOT_REMAP[i]] + state_.attitude.foot_force_offset(i)) * (1.0 - state_.ctrl.foot_force_filter) + state_.attitude.foot_force[i] * state_.ctrl.foot_force_filter;
        }
        for (int i = 0; i < NUM_DOF; ++i)
        {
            state_.attitude.joint_vel_q[i] = 0.4 * state_.attitude.joint_vel_q[i] + 0.6 * udp_comm_.state.motorState[JOINT_REMAP[i]].dq;
            state_.attitude.joint_pos_q[i] = udp_comm_.state.motorState[JOINT_REMAP[i]].q;
            state_.attitude.joint_force_q[i] = udp_comm_.state.motorState[JOINT_REMAP[i]].tauEst;
        }
        udp_received_ = true;
        state_.ctrl.sensor_received = true;
        state_.state_process();

        
        auto t2 = ros::Time::now();
        ros::Duration run_dt = t2 - t1;
        double interval_sec = (double)HARDWARE_FEEDBACK_INTERVEL_MS / 1000.0;
        if (interval_sec > run_dt.toSec())
        {
            ros::Duration(interval_sec - run_dt.toSec()).sleep();
        }
    }
}

void RealDog::send_ros_info()
{
    while (thread_distructed_ == false)
    {
        info_to_ros_.publish_state(state_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void RealDog::write_log()
{
    while (thread_distructed_ == false)
    {
        dog_control_.write_log(state_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


bool RealDog::update_foot_forces_grf(double dt)
{
    if (!state_estimate_.is_inited())
    {
        return false;
    }
    dog_control_.compute_grf(state_, dt); 
    return true;
}

bool RealDog::main_update(double dt)
{
    if (dog_control_.stand_ctrl(state_))
    {
        dog_control_.process_doggy_cmd(state_, dt);
    }
    if (!state_estimate_.is_inited())
    {
        state_estimate_.init_state(state_);
    }
    state_estimate_.update_estimation(state_, dt);
    dog_control_.anti_slip(state_);
    dog_control_.robot_fall_detect(state_);
    dog_control_.leg_kinematic_control(state_, dt);
    // static int t = 0;
    // t++;
    // if(t % 10 == 0)
    // {
    //     info_to_ros_.publish_state(state_);
    // }
    // if(t % 10 == 5)
    // {
    //     dog_control_.write_log(state_);
    // }
    
}

bool RealDog::send_cmd()
{
    dog_control_.compute_joint_torques(state_);
    state_.ctrl.joint_torques_set += friction_compensation_.compute_friction_compensation(state_.attitude.joint_vel_q);
    udp_comm_.cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for (int i = 0; i < NUM_DOF; i++)
    {
        if (state_.ctrl.stand_status == 0)
        {
            udp_comm_.cmd.motorCmd[i].mode = 0x00;
            udp_comm_.cmd.motorCmd[i].tau = 0;
        }
        else
        {
            udp_comm_.cmd.motorCmd[i].mode = 0x0A;
            if(abs(state_.ctrl.joint_torques_set(JOINT_REMAP[i])) > state_.ctrl.max_torque)
            {
                // std::cout << "reach maximum torque! " << i << ", " << abs(state_.ctrl.joint_torques_set(JOINT_REMAP[i])) << std::endl;
                
            }
            udp_comm_.cmd.motorCmd[i].tau = Utils::clamp(state_.ctrl.joint_torques_set(JOINT_REMAP[i]), -state_.ctrl.max_torque, state_.ctrl.max_torque);
        }

        udp_comm_.cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // shut down position control
        udp_comm_.cmd.motorCmd[i].Kp = 0;
        udp_comm_.cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // shut down velocity control
        udp_comm_.cmd.motorCmd[i].Kd = 0;
    }
    udp_comm_.send();
}

void RealDog::wait_for_udp_received()
{
    static int wait_counter = 0;
    while (udp_received_ == false)
    {
        wait_counter++;
        if (wait_counter > 1000)
        {
            if (wait_counter % 1000 == 0)
            {
                std::cout << std::setprecision(2) << "waiting for udp..." << std::endl;
            }
        }
        ros::Duration(0.001).sleep();
    }
    std::cout << std::setprecision(2) << "udp data received." << std::endl;
}


template <typename T>
void RealDog::log(spdlog::level::level_enum lvl, const T &msg)
{
    dog_control_.log(lvl ,msg);
}
template <typename... Args>
void RealDog::log(spdlog::level::level_enum lvl, spdlog::format_string_t<Args...> fmt, Args &&...args)
{
    dog_control_.log(lvl, fmt, args...);
}


void RealDog::receive_vio_data()
{
    vio_udp_.UDPRecv();
    static bool first_reseived = true;
    if (first_reseived && state_.attitude.com_quat.matrix() != Eigen::Matrix3d::Identity() && !vio_udp_.get_position().isZero())
    {
        state_.attitude.quat_vio_offset = state_.attitude.com_quat * vio_udp_.get_orientation();
        // std::cout << "vio_udp_.get_orientation()" << std::endl;
        // std::cout << vio_udp_.get_orientation().matrix() << std::endl;
        first_reseived = false;
        state_.attitude.pos_vio_offset = state_.attitude.com_pos - state_.attitude.quat_vio_offset * vio_udp_.get_position();
        // std::cout << "state_.attitude.quat_vio_offset.matrix()" << std::endl;
        // std::cout << state_.attitude.quat_vio_offset.matrix() << std::endl;
    }
    // std::cout << vio_udp_.get_position().transpose() << std::endl;
    state_.attitude.pos_vio = state_.attitude.quat_vio_offset * vio_udp_.get_position() + state_.attitude.pos_vio_offset;
    state_.attitude.quat_vio = state_.attitude.quat_vio_offset * vio_udp_.get_orientation();

}
void UDPLowComm::send_init()
{
    cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for (int i = 0; i < NUM_DOF; i++)
    {
        cmd.motorCmd[i].mode = 0x0A;                      // motor switch to servo (PMSM) mode
        cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // 禁止位置环
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // 禁止速度环
        cmd.motorCmd[i].Kd = 0;
        cmd.motorCmd[i].tau = 0;
    }
    send();
}


