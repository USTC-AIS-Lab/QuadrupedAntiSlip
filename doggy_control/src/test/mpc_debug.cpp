#include "mpc_controller.h"
#include "mpc_compact_controller.h"
#include "qp_controller.h"
#include "gait/walk_gait.h"
#include "InfoToROS.h"

#define USE_MPC 1
typedef std::unique_ptr<BaseGait> GaitPtr;

void test(DogState &state, double yaw)
{
    GaitPtr gait_ptr = std::make_unique<WalkGait>();
    state.ctrl.sensor_received = true;
    double dt = 0.002;
    Eigen::Vector3d inital_pos = Eigen::Vector3d(0, 0, 0.15);
    Eigen::Vector3d pos_set_offset = Eigen::Vector3d(0.0, 0, 0.15);
    Eigen::Vector3d inital_euler = Eigen::Vector3d(0, 0, yaw);
    Eigen::Vector3d euler_set_offset = Eigen::Vector3d(0, 0, 0);
    state.attitude.com_quat = Eigen::AngleAxisd(inital_euler(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(inital_euler(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(inital_euler(0), Eigen::Vector3d::UnitX());
    state.ctrl.com_euler_set = inital_euler + euler_set_offset;
    state.attitude.com_pos = inital_pos;
    state.attitude.com_lin_vel = Eigen::Vector3d(0.0, 0, 0);
    state.ctrl.com_lin_vel_set = Eigen::Vector3d(0.0, 0.0, 0.0);
    state.ctrl.com_ang_vel_set = Eigen::Vector3d(0.0, 0.0, 0.0);
    state.ctrl.com_ang_vel_r_set = Eigen::Vector3d(0.0, 0.0, 0.0);
    state.attitude.joint_pos_q << 0.162668, 0.822337, -1.83411, -0.147463, 0.821221, -1.83381, 0.163412, 0.826852, -1.84372, -0.150993, 0.825153, -1.84135;
    state.attitude.foot_force << 100, 100, 100, 100;
    state.state_process();
    pos_set_offset = state.attitude.com_rot_mat * pos_set_offset;
    state.ctrl.com_pos_set << inital_pos + pos_set_offset;
    gait_ptr->moving_plan = false;
    gait_ptr->compute_swing_legs_kinematics(state, dt);

    for (int i = 0; i < 4; ++i)
    {
        Eigen::Vector3d target = state.attitude.hip_pos.block<3, 1>(0, i);
        target(2) = 0.01;
        Eigen::Vector3d foot_pose_r = state.attitude.com_rot_mat.transpose() * (target - state.attitude.com_pos);
        foot_pose_r += state.attitude.com_rot_mat.transpose() * Eigen::Vector3d(0.0, 0, 0);
        state.attitude.joint_pos_q.segment<3>(i * 3) = state.a1_kin.ik(foot_pose_r, i);
    }
    state.state_process();
    gait_ptr->moving_plan = true;
    gait_ptr->compute_swing_legs_kinematics(state, dt);
    

#if USE_MPC
    MPCController controller = MPCController(gait_ptr);
    // MPCCompactController controller = MPCCompactController(gait_ptr);
#else
    QPController controller;
#endif
    for(int i = 0; i < 1; ++i)
    {
        state.ctrl.foot_forces_grf_set = controller.compute_ground_reaction_force(state, dt);
    }
    
#if USE_MPC
    // std::cout << "B:" << std::endl;
    // std::cout << controller.B << std::endl;
    // Eigen::Matrix<double, 13, 1> x_d = controller.B * controller.get_solution().segment<12>(MPC_STATE_DIM * (MPC_PLAN_HORIZON + 1));
    // std::cout << "B * u:" << std::endl;
    // std::cout << x_d << std::endl;

#endif
    // std::cout << "solution:" << std::endl;
    // std::cout << state.ctrl.foot_forces_grf_set << std::endl;
    std::cout << "solution in robot frame:" << std::endl;
    std::cout << state.ctrl.foot_forces_grf_set << std::endl;
#if USE_MPC
    controller.debug_cout_trajectory(5);
    controller.print_grf_plan(true);
// controller.debug_loss_analyse();
#endif
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_debug");
    ros::NodeHandle nh;
    InfoToROS info_to_ros(nh);
    DogState state;
    std::cout << "***** yaw = 0.0 *****" << std::endl;
    test(state, 0.0);
    // std::cout << "***** yaw = 1.57 *****" << std::endl;
    // test(state, 1.57);
    while (ros::ok())
    {
        info_to_ros.publish_state(state);
        ros::Rate(10).sleep();
    }
    return 0;
}