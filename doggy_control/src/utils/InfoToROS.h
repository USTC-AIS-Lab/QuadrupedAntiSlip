#ifndef INFO_TO_ROS_H_
#define INFO_TO_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include "dog_state.h"
#include "Utils.h"

class InfoToROS
{
public:
    InfoToROS(ros::NodeHandle &nh);
    void publish_state(DogState &state);
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_joint_angle_;
    ros::Publisher pub_imu_raw_;
    ros::Publisher pub_com_acc_r;
    ros::Publisher pub_estimated_pose_;
    ros::Publisher pub_com_pos_set_odom_;
    ros::Publisher pub_com_pos_set_com_offseted_odom_;
    ros::Publisher pub_grf_;
    ros::Publisher pub_debug_values_;
    ros::Publisher pub_mpc_reference_;
    ros::Publisher pub_mpc_prediction_;
    ros::Publisher pub_support_polygon_;
    ros::Publisher pub_virtual_support_polygon_;
    tf::TransformBroadcaster odom_tf_broadcaster_;
    ros::Publisher pub_foot_force_marker_array_;
    sensor_msgs::JointState joint_foot_msg_;


    void publish_joint_angle(DogState &state);
    void publish_imu_raw(DogState &state);
    void publish_com_acc_r(DogState &state);
    void publish_marker(DogState &state);
    void publish_estimated_pose(DogState &state);
    void publish_pose_set(DogState &state);
    void publish_pose_set_com_offseted(DogState &state);
    void publish_base_tf(DogState &state);
    void publish_pose_set_tf(DogState &state);
    void publish_abs_tf(DogState &state);
    void publish_world_tf(DogState &state);
    void publish_grf(DogState &state);
    void publish_debug_value(DogState &state);
    void publish_mpc_reference(DogState &state);
    void publish_mpc_prediction(DogState &state);
    void publish_support_polygon(DogState &state);
    void publish_virtual_support_polygon(DogState &state);
    void print_motor_torque(DogState &state);

    void add_foot_force_marker(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_foot_pose_set_marker(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_actuated_force_marker(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_foothold_set_marker(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_foot_pose_predicted_marker(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_virtual_support_polygon(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_support_polygon(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_com_set_projection(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_estimated_states(visualization_msgs::MarkerArray &marker_array, DogState &state);
    void add_foot_force_impedance(visualization_msgs::MarkerArray &marker_array, DogState &state);
};


#endif