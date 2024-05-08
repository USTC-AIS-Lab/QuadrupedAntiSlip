#ifndef _PROBE_GAIT_H_
#define _PROBE_GAIT_H_

#include "base_gait.h"

class ProbeGait : public BaseGait
{
public:
    ProbeGait();
    void compute_swing_legs_kinematics(DogState &state, double dt);
    void gait_counter_reset();
    double get_contact_plan(double dt, double window, int leg_index);
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> predict_foot_poses(Eigen::VectorXd reference_trajectory, int state_dim, DogState &state, double dt);
    Eigen::Vector4d calculate_foot_support_weight();
    void gait_cmd_handler(int cmd);
    bool ready_to_engage() override;
    bool ready_to_detach() override;
    int get_probe_leg(){return probe_leg_;}
private:
    int probe_leg_; // the index of the leg to execute the probe
    bool is_probing; // true if is probing the ground
    bool start_probe; // set to true to start probing, keep true duing the whole gait process.
    bool finishing_probe; // set to true to finish probing
    int probe_tick; // the gait phase tick when the leg reach the probe position.
    Eigen::Vector3d probe_foot_offset_abs;
    double foot_height;
    void compute_foot_kinematics(DogState &state);
    void counter_contact_plan(DogState &state);
    BezierUtils bezierUtil;
};


#endif