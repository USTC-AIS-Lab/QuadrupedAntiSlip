#ifndef DOG_STATE_H
#define DOG_STATE_H

#include <Eigen/Dense>
#include "dog_params.h"
#include <ros/ros.h>
#include "doggy_msgs/DoggyMove.h"
#include "a1_kinematics.h"
#include "geometry_msgs/PoseArray.h"
#include "slip_info.h"

// FL, FR, RL, RR

class EsitimatedState
{
public:
    EsitimatedState()
    {
        com_pos.setZero();
        foot_pos.setZero();
        root_var.setZero();
        foot_var.setZero();
    }
    Eigen::Vector3d com_pos;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos;
    Eigen::Matrix<double, 3, 3> root_var;
    Eigen::Matrix<double, 3*NUM_LEG, 3*NUM_LEG> foot_var;
};

class AttitudeState
{
public:
    AttitudeState()
    {
        reset();
    }
    Eigen::Vector4d get_contact_state_vec(){return Eigen::Vector4d((int)is_contact[0],(int)is_contact[1],(int)is_contact[2],(int)is_contact[3]);}
    void reset();

    void reset_from_rosparam();
    void compute_support_polygon(Eigen::Vector4d leg_weights);
    Eigen::Quaterniond com_quat;// measured
    Eigen::Vector3d imu_acc; // measured
    Eigen::Vector4d foot_force; // measured

    Eigen::Vector3d com_ang_vel_r; // measured
    Eigen::Vector3d com_ang_vel_r_filtered; // 
    Eigen::Matrix3d com_rot_mat; // calculated, p_w = mat * p_r
    Eigen::Matrix3d com_rot_mat_z; // calculated, abs to world
    Eigen::Matrix3d com_rot_mat_r_to_abs; // calculated
    Eigen::Matrix3d euler_to_ang_vel_mapping;
    Eigen::Vector3d com_euler; // calculated, roll, pitch, yaw
    Eigen::Vector3d com_ang_vel;  // calculated
    Eigen::Vector3d com_ang_vel_filtered; // 
    Eigen::Vector3d com_acc_r; // calculated
    Eigen::Matrix<double, 3, NUM_LEG> com_pos_four_cols;
    

    Eigen::Vector3d com_lin_vel; // estimated
    Eigen::Vector3d com_lin_vel_filtered;
    Eigen::Vector3d com_lin_vel_r; //estimated
    Eigen::Vector3d com_lin_vel_r_filtered;
    Eigen::Vector3d com_lin_vel_estimated; //estimated
    Eigen::Vector3d com_pos; // estimated
    Eigen::Vector3d com_pos_estimated;

    Eigen::Vector3d estimated_terrain_euler;
    double terrain_angle_filter;
    
    Eigen::Vector4d foot_force_offset;
    bool is_contact[NUM_LEG];
    bool early_contact[NUM_LEG];

    Eigen::Vector3d pos_vio;
    Eigen::Vector3d pos_vio_offset;
    Eigen::Quaterniond quat_vio;
    Eigen::Quaterniond quat_vio_offset;

    Eigen::Matrix<double, NUM_DOF, 1> joint_pos_q; //measured
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel_q; //measured
    Eigen::Matrix<double, NUM_DOF, 1> joint_force_q; //measured

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_r; // calculated in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_r;  // calculated
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_filtered;  // calculated
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel;  // calculated
    Eigen::Matrix<double, 3, NUM_LEG> last_foot_contact; // in world frame
    Eigen::Matrix<double, 12, 12> j_foot_r; // calculated

    
    Eigen::Matrix<double, 3, NUM_LEG> hip_pos_abs; // calculated
    Eigen::Matrix<double, 3, NUM_LEG> hip_pos_r; // calculated
    Eigen::Matrix<double, 3, NUM_LEG> hip_pos; // calculated

    Eigen::Matrix<double, 3, NUM_LEG> support_polygon; 
    Eigen::Matrix<double, 3, NUM_LEG> virtual_support_polygon; 

    double robot_mass;
    Eigen::Matrix3d trunk_inertia;
    std::vector<double> debug_value;
    EsitimatedState estimated_state;
};

class CtrlState
{
public:
    CtrlState()
    {
        reset();
    }
    void reset();
    void reset_from_rosparam();
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques_set;
    Eigen::Vector3d com_euler_set;
    Eigen::Vector3d com_pos_set;
    
    Eigen::Vector3d com_lin_vel_set;
    Eigen::Vector3d com_lin_vel_r_set; 
    Eigen::Vector3d com_lin_vel_actuated_set;
    
    Eigen::Vector3d com_ang_vel_set; 
    Eigen::Vector3d com_ang_vel_r_set; 
    Eigen::Vector3d com_ang_vel_actuated_set;

    Eigen::Vector3d com_offset_abs;
    Eigen::Vector3d com_set; // com_pos_set with offset
    double yaw_offset_set;
    //vec_abs = state_.attitude.com_rot_mat_z.transpose() * state_.attitude.com_rot_mat * vec_r
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_set;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_r_set;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_set;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_r_set;
    Eigen::Matrix<double, 3, NUM_LEG> foot_target_set;
    
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin_set;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf_set;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_imp_set;

    double max_lin_acc;
    double max_ang_acc;

    double default_body_height;
    bool plan_contact[NUM_LEG];
    double max_torque;
    double foot_force_filter;
    double foot_contact_threshold;
    int safe_power_level;
    bool udp_online; //has udp signal
    

    bool safety_halt; // be true when the robot fall
    double tilt_threshold;
    doggy_msgs::DoggyMove doggy_cmd;
    doggy_msgs::DoggyMove doggy_cmd_last;
    bool doggy_cmd_received;
    bool sensor_received;
    int stand_status; // 0: motor break, 1:laying_down, 2:standing_up, 3: standinng

    geometry_msgs::PoseArray mpc_reference;
    geometry_msgs::PoseArray mpc_prediction;
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> predicted_foot_poses;
};

class DogState
{
public:
    DogState()
    {
        reset();
    }
    void reset();
    void state_process(); // process the state from raw sensor input
    bool contact(int i);
    void contact_estimate();
    void terrain_angle_estimate();
    A1Kinematics a1_kin;
    AttitudeState attitude;
    CtrlState ctrl;
    SlipInfo slip_info;

};

#endif
