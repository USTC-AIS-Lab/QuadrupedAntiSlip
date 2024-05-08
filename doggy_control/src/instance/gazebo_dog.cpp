#include "gazebo_dog.h"

GazeboDog::GazeboDog(ros::NodeHandle &nh) : nh_(nh), info_to_ros_(nh)
{
    doggy_cmd_sub_ = nh_.subscribe("/doggy_cmd", 100, &GazeboDog::doggy_cmd_callback, this);

    pub_joint_cmd[0] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    pub_joint_cmd[1] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    pub_joint_cmd[2] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);
    pub_joint_cmd[3] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    pub_joint_cmd[4] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    pub_joint_cmd[5] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);
    pub_joint_cmd[6] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    pub_joint_cmd[7] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    pub_joint_cmd[8] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);
    pub_joint_cmd[9] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    pub_joint_cmd[10] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    pub_joint_cmd[11] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);

    sub_joint_msg[0] = nh_.subscribe("/a1_gazebo/FL_hip_controller/state", 2, &GazeboDog::FL_hip_state_callback, this);
    sub_joint_msg[1] = nh_.subscribe("/a1_gazebo/FL_thigh_controller/state", 2, &GazeboDog::FL_thigh_state_callback, this);
    sub_joint_msg[2] = nh_.subscribe("/a1_gazebo/FL_calf_controller/state", 2, &GazeboDog::FL_calf_state_callback, this);
    sub_joint_msg[3] = nh_.subscribe("/a1_gazebo/FR_hip_controller/state", 2, &GazeboDog::FR_hip_state_callback, this);
    sub_joint_msg[4] = nh_.subscribe("/a1_gazebo/FR_thigh_controller/state", 2, &GazeboDog::FR_thigh_state_callback, this);
    sub_joint_msg[5] = nh_.subscribe("/a1_gazebo/FR_calf_controller/state", 2, &GazeboDog::FR_calf_state_callback, this);
    sub_joint_msg[6] = nh_.subscribe("/a1_gazebo/RL_hip_controller/state", 2, &GazeboDog::RL_hip_state_callback, this);
    sub_joint_msg[7] = nh_.subscribe("/a1_gazebo/RL_thigh_controller/state", 2, &GazeboDog::RL_thigh_state_callback, this);
    sub_joint_msg[8] = nh_.subscribe("/a1_gazebo/RL_calf_controller/state", 2, &GazeboDog::RL_calf_state_callback, this);
    sub_joint_msg[9] = nh_.subscribe("/a1_gazebo/RR_hip_controller/state", 2, &GazeboDog::RR_hip_state_callback, this);
    sub_joint_msg[10] = nh_.subscribe("/a1_gazebo/RR_thigh_controller/state", 2, &GazeboDog::RR_thigh_state_callback, this);
    sub_joint_msg[11] = nh_.subscribe("/a1_gazebo/RR_calf_controller/state", 2, &GazeboDog::RR_calf_state_callback, this);

    sub_foot_contact_msg[0] = nh_.subscribe("/visual/FL_foot_contact/the_force", 2, &GazeboDog::FL_foot_contact_callback, this);
    sub_foot_contact_msg[1] = nh_.subscribe("/visual/FR_foot_contact/the_force", 2, &GazeboDog::FR_foot_contact_callback, this);
    sub_foot_contact_msg[2] = nh_.subscribe("/visual/RL_foot_contact/the_force", 2, &GazeboDog::RL_foot_contact_callback, this);
    sub_foot_contact_msg[3] = nh_.subscribe("/visual/RR_foot_contact/the_force", 2, &GazeboDog::RR_foot_contact_callback, this);

    nh_.getParam("use_ground_truth_pose", use_ground_truth_pose);
    sub_imu_msg = nh_.subscribe("/trunk_imu", 1, &GazeboDog::imu_callback, this);
    if(use_ground_truth_pose)
    {
        ground_truth_odom = nh_.subscribe("/body_pose_ground_truth", 1, &GazeboDog::grund_truth_pose_callback, this);
    }
    else
    {
        sub_pose_msg = nh_.subscribe("/torso_odom", 1, &GazeboDog::pose_callback, this);
    }
    vio_odom_ = nh_.subscribe("/camera_pose_ground_truth", 1, &GazeboDog::vio_odm_callback, this);
    

}

bool GazeboDog::update_foot_forces_grf(double dt)
{
    dog_control_.compute_grf(state_, dt);
    return true;
}

bool GazeboDog::main_update(double dt)
{
    static int start_delay = 0;
    if (!state_estimate_.is_inited())
    {
        start_delay ++;
        if(start_delay == 10)
        {
            state_estimate_.init_state(state_);
        }
    }
    else
    {
        state_.state_process();
        if(!use_ground_truth_pose)
        {
            state_estimate_.update_estimation(state_, dt);
        }
        state_.ctrl.sensor_received = true;
    }
    if (dog_control_.stand_ctrl(state_))
    {
        dog_control_.process_doggy_cmd(state_, dt);
    }
    dog_control_.leg_kinematic_control(state_, dt);
    dog_control_.anti_slip(state_);
    static int i;
    if(++i % 5 == 0)
    {
        info_to_ros_.publish_state(state_);
    }
    
}

bool GazeboDog::send_cmd()
{
    dog_control_.compute_joint_torques(state_);
    unitree_legged_msgs::LowCmd low_cmd;
    for (int i = 0; i < NUM_DOF; i++)
    {
        low_cmd.motorCmd[i].q = 0;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kp = 0;
        low_cmd.motorCmd[i].Kd = 0;
        if (state_.ctrl.stand_status == 0)
        {
            low_cmd.motorCmd[i].mode = 0x00;
            low_cmd.motorCmd[i].tau = 0;
        }
        else
        {
            low_cmd.motorCmd[i].mode = 0x0A;
            low_cmd.motorCmd[i].tau = Utils::clamp(state_.ctrl.joint_torques_set(i), -state_.ctrl.max_torque, state_.ctrl.max_torque);
        }
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
        // udp_comm_.cmd.motorCmd[i].tau = state_.ctrl.joint_torques_set(JOINT_REMAP[i]);
    }
    return true;
}

// callback functions
void GazeboDog::pose_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    // update
    state_.attitude.com_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                   odom->pose.pose.orientation.x,
                                                   odom->pose.pose.orientation.y,
                                                   odom->pose.pose.orientation.z) ;
    // std::cout << "state_.attitude.joint_pos_q" << std::endl;
    // std::cout << state_.attitude.joint_pos_q.transpose() << std::endl;
    // std::cout << "state_.attitude.com_quat" << std::endl;
    // std::cout << Eigen::Vector4d(odom->pose.pose.orientation.w,
    //                              odom->pose.pose.orientation.x,
    //                              odom->pose.pose.orientation.y,
    //                              odom->pose.pose.orientation.z).transpose()
    //           << std::endl;
    // std::cout << "state_.attitude.com_pos" << std::endl;
    // std::cout << state_.attitude.com_pos << std::endl;
    // std::cout << "state_.attitude.com_lin_vel" << std::endl;
    // std::cout << state_.attitude.com_lin_vel << std::endl;   
}


void GazeboDog::vio_odm_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    static bool first_reseived = true;
    if(first_reseived && state_.attitude.com_quat.matrix() != Eigen::Matrix3d::Identity())
    {
        state_.attitude.quat_vio_offset = state_.attitude.com_quat *
                                          Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                             odom->pose.pose.orientation.x,
                                                             odom->pose.pose.orientation.y,
                                                             odom->pose.pose.orientation.z).inverse();
        first_reseived = false;
        state_.attitude.pos_vio_offset = state_.attitude.com_pos -
                                         state_.attitude.quat_vio_offset *
                                             Eigen::Vector3d(odom->pose.pose.position.x,
                                                             odom->pose.pose.position.y,
                                                             odom->pose.pose.position.z);
        //std::cout << "offset set!" << std::endl;
    }
    state_.attitude.pos_vio = state_.attitude.quat_vio_offset *
                                  Eigen::Vector3d(odom->pose.pose.position.x,
                                                  odom->pose.pose.position.y,
                                                  odom->pose.pose.position.z) +
                              state_.attitude.pos_vio_offset;
    state_.attitude.quat_vio = state_.attitude.quat_vio_offset *
                               Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                  odom->pose.pose.orientation.x,
                                                  odom->pose.pose.orientation.y,
                                                  odom->pose.pose.orientation.z);
}
void GazeboDog::grund_truth_pose_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    state_.attitude.com_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                   odom->pose.pose.orientation.x,
                                                   odom->pose.pose.orientation.y,
                                                   odom->pose.pose.orientation.z);
    state_.attitude.com_pos_estimated = Eigen::Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    state_.attitude.com_lin_vel_estimated = Eigen::Vector3d(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);
    state_.attitude.com_pos = state_.attitude.com_pos_estimated;
    state_.attitude.com_lin_vel = state_.attitude.com_lin_vel_estimated;
    state_.attitude.com_lin_vel_r = state_.attitude.com_rot_mat.transpose() * state_.attitude.com_lin_vel;

    state_.attitude.com_ang_vel_r = state_.attitude.com_rot_mat.transpose() * Eigen::Vector3d(odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z);                  
}

void GazeboDog::imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
    state_.attitude.imu_acc = Eigen::Vector3d(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    if(!use_ground_truth_pose)
    {
        state_.attitude.com_ang_vel_r = Eigen::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    }
    // state_.attitude.com_quat = Eigen::Quaterniond(imu->orientation.w,
    //                                               imu->orientation.x,
    //                                               imu->orientation.y,
    //                                               imu->orientation.z);
}

void GazeboDog::doggy_cmd_callback(const doggy_msgs::DoggyMove &msg)
{
    state_.ctrl.doggy_cmd = msg;
    state_.ctrl.doggy_cmd_received = true;
}

void GazeboDog::FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[0] = a1_joint_state.q;
    state_.attitude.joint_vel_q[0] = a1_joint_state.dq;
}

void GazeboDog::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[1] = a1_joint_state.q;
    state_.attitude.joint_vel_q[1] = a1_joint_state.dq;
}

void GazeboDog::FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[2] = a1_joint_state.q;
    state_.attitude.joint_vel_q[2] = a1_joint_state.dq;
}

// FR
void GazeboDog::FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[3] = a1_joint_state.q;
    state_.attitude.joint_vel_q[3] = a1_joint_state.dq;
}

void GazeboDog::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[4] = a1_joint_state.q;
    state_.attitude.joint_vel_q[4] = a1_joint_state.dq;
}

void GazeboDog::FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[5] = a1_joint_state.q;
    state_.attitude.joint_vel_q[5] = a1_joint_state.dq;
}

// RL
void GazeboDog::RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[6] = a1_joint_state.q;
    state_.attitude.joint_vel_q[6] = a1_joint_state.dq;
}

void GazeboDog::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[7] = a1_joint_state.q;
    state_.attitude.joint_vel_q[7] = a1_joint_state.dq;
}

void GazeboDog::RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[8] = a1_joint_state.q;
    state_.attitude.joint_vel_q[8] = a1_joint_state.dq;
}

// RR
void GazeboDog::RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[9] = a1_joint_state.q;
    state_.attitude.joint_vel_q[9] = a1_joint_state.dq;
}

void GazeboDog::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[10] = a1_joint_state.q;
    state_.attitude.joint_vel_q[10] = a1_joint_state.dq;
}

void GazeboDog::RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state)
{
    state_.attitude.joint_pos_q[11] = a1_joint_state.q;
    state_.attitude.joint_vel_q[11] = a1_joint_state.dq;
}

// foot contact force
void GazeboDog::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force)
{
    state_.attitude.foot_force[0] = Eigen::Vector3d(force.wrench.force.x, force.wrench.force.y, force.wrench.force.z).norm() * 2;
}

void GazeboDog::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force)
{
    state_.attitude.foot_force[1] = Eigen::Vector3d(force.wrench.force.x, force.wrench.force.y, force.wrench.force.z).norm() * 2;
}

void GazeboDog::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force)
{
    state_.attitude.foot_force[2] = Eigen::Vector3d(force.wrench.force.x, force.wrench.force.y, force.wrench.force.z).norm() * 2;
}

void GazeboDog::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force)
{
    state_.attitude.foot_force[3] = Eigen::Vector3d(force.wrench.force.x, force.wrench.force.y, force.wrench.force.z).norm() * 2;
}
