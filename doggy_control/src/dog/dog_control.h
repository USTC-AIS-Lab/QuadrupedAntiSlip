#ifndef DOG_CONTROL_H
#define DOG_CONTROL_H
#include <iostream>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

#include "dog_params.h"
#include "dog_state.h"
#include "utils/Utils.h"

#include "base_controller.h"
#include "qp_controller.h"
#include "mpc_controller.h"

#include "gait/base_gait.h"
#include "gait/walk_gait.h"
#include "gait/pee.h"

#include "anti_slip.h"
#include "swing_leg_controller.h"
#include "RobotLog.h"


// FL, FR, RL, RR
class DogControl
{
public:
    typedef std::unique_ptr<BaseController> ControllerPtr;
    typedef std::unique_ptr<BaseGait> GaitPtr;
    DogControl();
    void compute_joint_torques(DogState &state);
    void leg_kinematic_control(DogState &state, double dt);
    void robot_fall_detect(DogState &state);
    void process_doggy_cmd(DogState &state, double dt);
    void compute_grf(DogState &state, double dt);
    bool stand_ctrl(DogState &state);
    void anti_slip(DogState &state);
    void write_log(DogState &state);
    template <typename T>
    void log(spdlog::level::level_enum lvl, const T &msg)
    {
        robot_log_.log(lvl, msg);
    }

    template <typename... Args>
    void log(spdlog::level::level_enum lvl, spdlog::format_string_t<Args...> fmt, Args &&...args)
    {
        robot_log_.log(lvl,fmt, args...);
    }

private:
    ControllerPtr controller_ptr_;
    GaitPtr gait_ptr_;
    SwingLegController swing_leg_controller;
    RobotLog robot_log_;
    AntiSlip anti_slip_;

    void com_offset_ctrl(DogState &state);
    void process_walk_mode_cmd(DogState &state, double dt);
    void process_stance_mode_cmd(DogState &state, double dt);
    void process_gait_switch_cmd(DogState &state, double dt);
};


#endif
