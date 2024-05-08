#ifndef _PEE_H_
#define _PEE_H_

#include "base_gait.h"

class Pee : public BaseGait
{
public:
    Pee();
    void compute_swing_legs_kinematics(DogState &state, double dt);
    void gait_counter_reset();
    double get_contact_plan(double dt, double window, int leg_index);
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> predict_foot_poses(Eigen::VectorXd reference_trajectory, int state_dim, DogState &state, double dt);
    Eigen::Vector4d calculate_foot_support_weight();
    void gait_cmd_handler(int cmd);
private:
    int pee_leg;
    bool peeing;
    bool start;
    bool finishing;
    int pee_time;
    void counter_contact_plan(DogState &state);
    BezierUtils bezierUtil;
    Eigen::Vector3d pee_foot_offset_abs;
    void compute_foot_kinematics(DogState &state);
};

#endif