#ifndef _BASE_GAIT_H_
#define _BASE_GAIT_H_

#include "dog_params.h"
#include "dog_state.h"
#include "Utils.h"
#include <chrono>
#include <string>
#include <Eigen/Dense>


class BaseGait
{
public:
    BaseGait()
    {
        gait_name_ = "base";
        gait_spansteps_.setZero();
        moving_plan = false;
        foot_pos_start_r.setZero();
        foot_pos_target_abs.setZero();
        foot_pos_start_abs.setZero(); 
        foot_pos_first_contact.setZero();
        com_offset_ratio = 0;
        body_height_ = 0;
        max_vel.setZero();
        for(int i = 0; i < NUM_LEG; ++i)
        {
            is_moving[i] = false;
        }
    }
    virtual void compute_swing_legs_kinematics(DogState &state, double dt) = 0;
    virtual void gait_counter_reset() = 0;
    virtual double get_contact_plan(double dt, double window, int leg_index) = 0;
    virtual std::vector<Eigen::Matrix<double, 3, NUM_LEG>> predict_foot_poses(Eigen::VectorXd reference_trajectory, int state_dim, DogState &state, double dt) = 0;
    virtual void gait_cmd_handler(int cmd) = 0;
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> get_foot_pose_prediction(){return foot_pose_predicted;}
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start_r;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_first_contact;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    virtual Eigen::Vector4d calculate_foot_support_weight() = 0;
    Eigen::Vector3d get_max_vel(){return max_vel;};
    virtual bool ready_to_detach(){return !(moving_plan || is_moving[0] || is_moving[1] || is_moving[2] || is_moving[3]);}
    virtual bool ready_to_engage(){return true;} // for overriding
    virtual bool is_in_swing_phase(int leg_index){return gait_counter_(leg_index) < swing_spansteps_(leg_index);}
    std::string get_gait_name(){return gait_name_;}
    Eigen::Vector4i get_gait_counter() {return gait_counter_;}; // all using 4000 as the max counter vel
    Eigen::Vector4i get_gait_spansteps() {return gait_spansteps_;};
    Eigen::Vector4i get_swing_spansteps() {return swing_spansteps_;};
    Eigen::Vector4i get_stance_spansteps() {return stance_spansteps_;};
    double com_offset_ratio;
    bool moving_plan; // is ture when the robot is set to moving mode (but some of the feet may be still)
    bool is_moving[4]; // whether a specific foot is moving
    double get_body_height(DogState &state){return body_height_ > 0 ? body_height_ : state.ctrl.default_body_height;}
protected:
    std::string gait_name_;
    Eigen::Vector4i gait_counter_; // all using 4000 as the max counter vel
    Eigen::Vector4i initial_counter_;
    Eigen::Vector4i gait_spansteps_;
    Eigen::Vector4i swing_spansteps_;
    Eigen::Vector4i stance_spansteps_;
    double body_height_;
    
    Eigen::Vector3d max_vel;
    std::vector<Eigen::Matrix<double, 3, NUM_LEG>> foot_pose_predicted;
    virtual int predict_gait_counter(double dt, int leg_index, bool mode=true)
    {
        int step = dt * 1000.0 / MAIN_UPDATE_INTERVEL_MS;
        int gait_counter_predicted;
        gait_counter_predicted = (gait_counter_(leg_index) + step);
        if (mode)
        {
            gait_counter_predicted = gait_counter_predicted % (gait_spansteps_(leg_index));
        }

        return gait_counter_predicted;
    }
};


#endif
