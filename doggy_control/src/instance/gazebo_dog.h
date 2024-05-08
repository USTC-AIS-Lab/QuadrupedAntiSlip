#ifndef __GAZEBO_DOG_H
#define __GAZEBO_DOG_H

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

#include <geometry_msgs/WrenchStamped.h>

#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>

// FL, FR, RL, RR
class GazeboDog
{
public:
    GazeboDog(ros::NodeHandle &nh);
    bool main_update(double dt);
    bool update_foot_forces_grf(double dt);
    bool send_cmd();
private:
    void doggy_cmd_callback(const doggy_msgs::DoggyMove &msg);
    void pose_callback(const nav_msgs::Odometry::ConstPtr &odom);
    void grund_truth_pose_callback(const nav_msgs::Odometry::ConstPtr &odom);
    void vio_odm_callback(const nav_msgs::Odometry::ConstPtr &odom);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);
    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);
    void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    bool use_ground_truth_pose;
    ros::NodeHandle nh_;
    DogState state_;
    DogControl dog_control_;
    StateEstimate state_estimate_;
    ros::Subscriber doggy_cmd_sub_;
    InfoToROS info_to_ros_;

    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf
    ros::Publisher pub_joint_cmd[12];
    ros::Subscriber sub_joint_msg[12];
    // 0, 1, 2, 3: FL, FR, RL, RR
    ros::Subscriber sub_foot_contact_msg[4];
    ros::Subscriber sub_pose_msg;
    ros::Subscriber sub_imu_msg;
    ros::Subscriber ground_truth_odom;
    ros::Subscriber vio_odom_;
};


#endif
