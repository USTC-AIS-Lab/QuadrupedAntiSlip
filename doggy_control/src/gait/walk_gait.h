#ifndef _WALK_GAIT_H_
#define _WALK_GAIT_H_

#include "base_gait.h"

class WalkGait : public BaseGait
{
public:
    WalkGait();
    void compute_swing_legs_kinematics(DogState &state, double dt);
    void gait_counter_reset();
    double get_contact_plan(double dt, double window, int leg_index);
    Eigen::Matrix<double, 3, NUM_LEG> predict_foot_pose(Eigen::Matrix<double, 3, NUM_LEG> hip_pos, Eigen::Vector3d com_lin_vel, Eigen::Vector3d com_lin_vel_set, Eigen::Vector3d com_ang_vel, Eigen::Vector3d com_ang_vel_set, Eigen::Vector4i counter_now);
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> predict_foot_poses(Eigen::VectorXd reference_trajectory, int state_dim, DogState &state, double dt);
    Eigen::Vector4d calculate_foot_support_weight();
    void gait_cmd_handler(int cmd);
    
private:
    double foot_delta_x_limit;
    double foot_delta_y_limit;
    bool foot_start_allocated[4];
    double foot_height;
    double touch_down;
    Eigen::Vector4i pregait_counter_;
    Eigen::Vector3d terrain_angle;
    Eigen::Vector4f initial_phase;
    Eigen::Vector4d swing_phase;
    // int predict_gait_counter(double dt, int leg_index, bool mode=true);
    void counter_contact_plan(DogState &state);
    void compute_foot_kinematics(DogState &state);
    bool is_contact_phase(int counter_now, int foot_num);
    Eigen::Matrix<double, 3, NUM_LEG> calculate_foot_pos_target(Eigen::Matrix<double, 3, NUM_LEG> hip_pos, Eigen::Vector3d terrain_angle, Eigen::Vector3d com_lin_vel, Eigen::Vector3d com_lin_vel_set, Eigen::Vector3d com_ang_vel, Eigen::Vector3d com_ang_vel_set, Eigen::Vector4i counter_now);
    BezierUtils bezierUtil;
    int predict_gait_counter(double dt, int leg_index, bool mode=true) override;
};

#endif
