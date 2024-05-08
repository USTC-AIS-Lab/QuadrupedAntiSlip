#include "InfoToROS.h"

InfoToROS::InfoToROS(ros::NodeHandle &nh) : nh_(nh)
{
    pub_joint_angle_ = nh.advertise<sensor_msgs::JointState>("/dog_info/joint_states", 1000);
    pub_imu_raw_ = nh.advertise<sensor_msgs::Imu>("/dog_info/imu", 1000);
    pub_com_acc_r = nh.advertise<sensor_msgs::Imu>("/dog_info/com_acc_r", 1000);
    // pub_joint_cmd_ = nh.advertise<sensor_msgs::JointState>("/real_dog/joint_torque_cmd", 100);
    pub_foot_force_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("/dog_info/foot_marker_array", 5000);
    pub_debug_values_ = nh.advertise<std_msgs::Float64MultiArray>("/dog_info/debug_info", 1000);
    pub_estimated_pose_ = nh.advertise<nav_msgs::Odometry>("/dog_info/estimated_body_pose", 1000);
    pub_com_pos_set_odom_ = nh.advertise<nav_msgs::Odometry>("/dog_info/pose_set", 1000);
    pub_com_pos_set_com_offseted_odom_ = nh.advertise<nav_msgs::Odometry>("/dog_info/pose_set_com_offset", 1000);
    pub_grf_ = nh.advertise<std_msgs::Float64MultiArray>("/dog_info/ground_reaction_force", 1000);
    pub_mpc_reference_ = nh.advertise<geometry_msgs::PoseArray>("/dog_info/mpc_reference", 1000);
    pub_mpc_prediction_ = nh.advertise<geometry_msgs::PoseArray>("/dog_info/prediction", 1000);
    pub_support_polygon_ = nh.advertise<geometry_msgs::PolygonStamped>("/dog_info/support_polygon", 1000);
    pub_virtual_support_polygon_ = nh.advertise<geometry_msgs::PolygonStamped>("/dog_info/pub_virtual_support_polygon", 1000);
    joint_foot_msg_.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    joint_foot_msg_.position.resize(NUM_DOF);
    joint_foot_msg_.velocity.resize(NUM_DOF);
    joint_foot_msg_.effort.resize(NUM_DOF);
}

void InfoToROS::publish_joint_angle(DogState &state)
{
    for (int i = 0; i < NUM_DOF; ++i)
    {
        joint_foot_msg_.position[i] = state.attitude.joint_pos_q[i];
        joint_foot_msg_.velocity[i] = state.attitude.joint_vel_q[i];
        joint_foot_msg_.effort[i] = state.ctrl.joint_torques_set[i];
    }
    joint_foot_msg_.header.stamp = ros::Time::now();
    pub_joint_angle_.publish(joint_foot_msg_);
}

void InfoToROS::publish_imu_raw(DogState &state)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base";
    imu_msg.angular_velocity.x = state.attitude.com_ang_vel_r(0);
    imu_msg.angular_velocity.y = state.attitude.com_ang_vel_r(1);
    imu_msg.angular_velocity.z = state.attitude.com_ang_vel_r(2);

    imu_msg.linear_acceleration.x = state.attitude.imu_acc(0);
    imu_msg.linear_acceleration.y = state.attitude.imu_acc(1);
    imu_msg.linear_acceleration.z = state.attitude.imu_acc(2);

    imu_msg.orientation.x = state.attitude.com_quat.x();
    imu_msg.orientation.y = state.attitude.com_quat.y();
    imu_msg.orientation.z = state.attitude.com_quat.z();
    imu_msg.orientation.w = state.attitude.com_quat.w();
    pub_imu_raw_.publish(imu_msg);
}

void InfoToROS::publish_com_acc_r(DogState &state)
{
    sensor_msgs::Imu com_acc_r_msg;
    com_acc_r_msg.header.stamp = ros::Time::now();
    com_acc_r_msg.header.frame_id = "base";
    com_acc_r_msg.angular_velocity.x = state.attitude.com_ang_vel_r(0);
    com_acc_r_msg.angular_velocity.y = state.attitude.com_ang_vel_r(1);
    com_acc_r_msg.angular_velocity.z = state.attitude.com_ang_vel_r(2);

    com_acc_r_msg.linear_acceleration.x = state.attitude.com_acc_r(0);
    com_acc_r_msg.linear_acceleration.y = state.attitude.com_acc_r(1);
    com_acc_r_msg.linear_acceleration.z = state.attitude.com_acc_r(2);

    pub_com_acc_r.publish(com_acc_r_msg);
}

void InfoToROS::add_foot_force_marker(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time();
        marker.ns = "foot_force";
        marker.id++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = state.attitude.foot_pos_r(0, i);
        marker.pose.position.y = state.attitude.foot_pos_r(1, i);
        marker.pose.position.z = state.attitude.foot_pos_r(2, i);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.707;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.707;
        marker.scale.x = double(state.attitude.foot_force(i)) * 0.002;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.97;
        marker.color.g = 0.0;
        marker.color.b = 0.07;
        marker_array.markers.emplace_back(marker);
    }
}

void InfoToROS::add_foot_pose_set_marker(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "foot_pos_set";
        marker.id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        if(state.slip_info.slip_state(i))
        {
            marker.pose.position.x = state.slip_info.foot_slip_spot(0, i);
            marker.pose.position.y = state.slip_info.foot_slip_spot(1, i);
            marker.pose.position.z = state.slip_info.foot_slip_spot(2, i);
        }
        else
        {
            marker.pose.position.x = state.ctrl.foot_pos_set(0, i);
            marker.pose.position.y = state.ctrl.foot_pos_set(1, i);
            marker.pose.position.z = state.ctrl.foot_pos_set(2, i);
        }
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.707;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.707;
        marker.scale.x = 0.05; // 0.05 is good, don't modify this.
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        if (state.contact(i))
        {
            if(state.slip_info.slip_state(i))
            {
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            else
            {
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            
        }
        else
        {
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        marker_array.markers.emplace_back(marker);
    }
}

void InfoToROS::add_actuated_force_marker(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time();
        marker.ns = "actuated_force";
        marker.id++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = state.attitude.foot_pos_r(0, i);
        marker.pose.position.y = state.attitude.foot_pos_r(1, i);
        marker.pose.position.z = state.attitude.foot_pos_r(2, i);
        if (state.contact(i))
        {
            Eigen::Quaterniond quaternion = Utils::xyz_pos_to_quaternion(state.ctrl.foot_forces_grf_set.block<3, 1>(0, i));
            marker.pose.orientation.x = quaternion.x();
            marker.pose.orientation.y = quaternion.y();
            marker.pose.orientation.z = quaternion.z();
            marker.pose.orientation.w = quaternion.w();
            marker.scale.x = double(state.ctrl.foot_forces_grf_set.block<3, 1>(0, i).norm()) * 0.004 + 0.001;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.2;
        }
        else
        {
            Eigen::Quaterniond quaternion = Utils::xyz_pos_to_quaternion(state.ctrl.foot_forces_kin_set.block<3, 1>(0, i));
            marker.pose.orientation.x = quaternion.x();
            marker.pose.orientation.y = quaternion.y();
            marker.pose.orientation.z = quaternion.z();
            marker.pose.orientation.w = quaternion.w();
            marker.scale.x = double(state.ctrl.foot_forces_kin_set.block<3, 1>(0, i).norm()) * 0.01 + 0.001;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.2;
            marker.color.b = 1.0;
        }

        marker_array.markers.emplace_back(marker);
    }
}

void InfoToROS::add_foothold_set_marker(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "foothold_set";
        marker.id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = state.ctrl.foot_target_set(0, i);
        marker.pose.position.y = state.ctrl.foot_target_set(1, i);
        marker.pose.position.z = state.ctrl.foot_target_set(2, i);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.707;
        marker.pose.orientation.w = 0.707;
        marker.scale.x = 0.06;
        marker.scale.y = 0.06;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        if (state.attitude.is_contact[i])
        {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.r = 0.8;
            marker.color.g = 0.8;
            marker.color.b = 0.8;
        }

        marker_array.markers.emplace_back(marker);
    }
}

void InfoToROS::add_foot_pose_predicted_marker(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    for (int j = 0; j < NUM_LEG; ++j)
    {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "predicted_foot_poses_" + std::to_string(j);
        marker.id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points.clear();
        for (int i = 0; i < state.ctrl.predicted_foot_poses.size(); ++i)
        {
            geometry_msgs::Point point;
            point.x = state.ctrl.predicted_foot_poses[i](0, j);
            point.y = state.ctrl.predicted_foot_poses[i](1, j);
            point.z = state.ctrl.predicted_foot_poses[i](2, j);
            marker.points.emplace_back(point);
        }
        marker.scale.x = 0.004;
        marker.scale.y = 0.001;
        marker.scale.z = 0.001;
        marker.color.a = 0.9;
        marker.color.r = 0.0;
        marker.color.g = 0.9;
        marker.color.b = 0.0;
        marker_array.markers.emplace_back(marker);
    }
}

void InfoToROS::add_virtual_support_polygon(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "virtual_support_polygon";
    marker.id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.clear();
    int index[6] = {0, 1, 3, 2, 0, 1};
    for (int i = 0; i < 6; ++i)
    {
        if(state.attitude.virtual_support_polygon(0, index[i]) != 0 && state.attitude.virtual_support_polygon(1, index[i]) != 0)
        {
            geometry_msgs::Point point;
            point.x = state.attitude.virtual_support_polygon(0, index[i]);
            point.y = state.attitude.virtual_support_polygon(1, index[i]);
            point.z = state.attitude.virtual_support_polygon(2, index[i]);
            marker.points.emplace_back(point);
        }
        
    }
    marker.scale.x = 0.006;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.a = 0.9;
    marker.color.r = 0.9;
    marker.color.g = 0.9;
    marker.color.b = 0.0;
    marker_array.markers.emplace_back(marker);
}

void InfoToROS::add_support_polygon(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "support_polygon";
    marker.id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.clear();
    int index[6] = {0, 1, 3, 2, 0, 1};
    for (int i = 0; i < 6; ++i)
    {
        if(state.attitude.support_polygon(0, index[i]) != 0 && state.attitude.support_polygon(1, index[i]) != 0)
        {
            geometry_msgs::Point point;
            point.x = state.attitude.support_polygon(0, index[i]);
            point.y = state.attitude.support_polygon(1, index[i]);
            point.z = state.attitude.support_polygon(2, index[i]);
            marker.points.emplace_back(point);
        }
        
    }
    marker.scale.x = 0.006;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.a = 0.9;
    marker.color.r = 0.0;
    marker.color.g = 0.9;
    marker.color.b = 0.0;
    marker_array.markers.emplace_back(marker);
}
void InfoToROS::add_com_set_projection(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "com_set_projection";
    marker.id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    Eigen::Vector3d com_set_projection = state.ctrl.com_set;
    marker.pose.position.x = com_set_projection(0);
    marker.pose.position.y = com_set_projection(1);
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.707;
    marker.pose.orientation.w = 0.707;
    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_array.markers.emplace_back(marker);
}

void InfoToROS::add_estimated_states(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "estimated_states";
        marker.id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = state.attitude.estimated_state.foot_pos(0, i);
        marker.pose.position.y = state.attitude.estimated_state.foot_pos(1, i);
        marker.pose.position.z = state.attitude.estimated_state.foot_pos(2, i);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::log(state.attitude.estimated_state.foot_var.block<3, 3>(3 * i, 3 * i)(0, 0) + 1) * 0.25;
        marker.scale.y = std::log(state.attitude.estimated_state.foot_var.block<3, 3>(3 * i, 3 * i)(1, 1) + 1) * 0.25;
        marker.scale.z = std::log(state.attitude.estimated_state.foot_var.block<3, 3>(3 * i, 3 * i)(2, 2) + 1) * 0.25;
        marker.color.a = 1.0 - std::erf(marker.scale.x + marker.scale.y + marker.scale.z);
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.emplace_back(marker);
    }

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "estimated_states";
    marker.id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = state.attitude.estimated_state.com_pos(0);
    marker.pose.position.y = state.attitude.estimated_state.com_pos(1);
    marker.pose.position.z = state.attitude.estimated_state.com_pos(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::log(state.attitude.estimated_state.root_var(0, 0) + 1) * 2.0;
    marker.scale.y = std::log(state.attitude.estimated_state.root_var(1, 1) + 1) * 2.0;
    marker.scale.z = std::log(state.attitude.estimated_state.root_var(2, 2) + 1) * 2.0;
    marker.color.a = 1.0 - std::erf(marker.scale.x + marker.scale.y + marker.scale.z);
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.emplace_back(marker);
}

void InfoToROS::add_foot_force_impedance(visualization_msgs::MarkerArray &marker_array, DogState &state)
{
    visualization_msgs::Marker marker;
    marker.id = marker_array.markers.size();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time();
        marker.ns = "impedance_force";
        marker.id++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = state.attitude.foot_pos_r(0, i);
        marker.pose.position.y = state.attitude.foot_pos_r(1, i);
        marker.pose.position.z = state.attitude.foot_pos_r(2, i);

        Eigen::Quaterniond quaternion = Utils::xyz_pos_to_quaternion(state.ctrl.foot_forces_imp_set.block<3, 1>(0, i));
        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();
        marker.scale.x = double(state.ctrl.foot_forces_imp_set.block<3, 1>(0, i).norm()) * 0.004 + 0.003;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.2;

            marker_array.markers.emplace_back(marker);
    }
}

void InfoToROS::publish_marker(DogState &state)
{
    visualization_msgs::MarkerArray marker_array_msg;
    add_foot_force_marker(marker_array_msg, state);
    add_foot_pose_set_marker(marker_array_msg, state);
    add_actuated_force_marker(marker_array_msg, state);
    add_foothold_set_marker(marker_array_msg, state);
    add_foot_pose_predicted_marker(marker_array_msg, state);
    add_support_polygon(marker_array_msg, state);
    add_virtual_support_polygon(marker_array_msg, state);
    add_com_set_projection(marker_array_msg, state);
    // add_estimated_states(marker_array_msg, state);
    add_foot_force_impedance(marker_array_msg, state);
    pub_foot_force_marker_array_.publish(marker_array_msg);
}

void InfoToROS::publish_estimated_pose(DogState &state)
{
    nav_msgs::Odometry estimated_odom;
    estimated_odom.header.frame_id = "world";
    estimated_odom.header.stamp = ros::Time::now();
    estimated_odom.child_frame_id = "base";
    estimated_odom.pose.pose.position.x = state.attitude.com_pos(0);
    estimated_odom.pose.pose.position.y = state.attitude.com_pos(1);
    estimated_odom.pose.pose.position.z = state.attitude.com_pos(2);
    estimated_odom.pose.pose.orientation.x = state.attitude.com_quat.x();
    estimated_odom.pose.pose.orientation.y = state.attitude.com_quat.y();
    estimated_odom.pose.pose.orientation.z = state.attitude.com_quat.z();
    estimated_odom.pose.pose.orientation.w = state.attitude.com_quat.w();
    // make sure com_lin_vel is in world frame
    estimated_odom.twist.twist.linear.x = state.attitude.com_lin_vel_estimated(0);
    estimated_odom.twist.twist.linear.y = state.attitude.com_lin_vel_estimated(1);
    estimated_odom.twist.twist.linear.z = state.attitude.com_lin_vel_estimated(2);

    estimated_odom.twist.twist.angular.x = state.attitude.com_ang_vel(0);
    estimated_odom.twist.twist.angular.y = state.attitude.com_ang_vel(1);
    estimated_odom.twist.twist.angular.z = state.attitude.com_ang_vel(2);
    pub_estimated_pose_.publish(estimated_odom);
}

void InfoToROS::publish_pose_set(DogState &state)
{
    nav_msgs::Odometry pose_set_odom;
    pose_set_odom.header.frame_id = "world";
    pose_set_odom.header.stamp = ros::Time::now();
    pose_set_odom.child_frame_id = "pose_set";
    pose_set_odom.pose.pose.position.x = state.ctrl.com_set(0);
    pose_set_odom.pose.pose.position.y = state.ctrl.com_set(1);
    pose_set_odom.pose.pose.position.z = state.ctrl.com_set(2);
    Eigen::Quaterniond quaternion_pos_set = Eigen::AngleAxisd(state.ctrl.com_euler_set(2) + state.ctrl.yaw_offset_set, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state.ctrl.com_euler_set(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(state.ctrl.com_euler_set(0), Eigen::Vector3d::UnitX());
    pose_set_odom.pose.pose.orientation.x = quaternion_pos_set.x();
    pose_set_odom.pose.pose.orientation.y = quaternion_pos_set.y();
    pose_set_odom.pose.pose.orientation.z = quaternion_pos_set.z();
    pose_set_odom.pose.pose.orientation.w = quaternion_pos_set.w();
    // make sure com_lin_vel is in world frame
    pose_set_odom.twist.twist.linear.x = state.ctrl.com_lin_vel_r_set(0);
    pose_set_odom.twist.twist.linear.y = state.ctrl.com_lin_vel_r_set(1);
    pose_set_odom.twist.twist.linear.z = state.ctrl.com_lin_vel_r_set(2);

    pose_set_odom.twist.twist.angular.x = state.ctrl.com_ang_vel_r_set(0);
    pose_set_odom.twist.twist.angular.y = state.ctrl.com_ang_vel_r_set(1);
    pose_set_odom.twist.twist.angular.z = state.ctrl.com_ang_vel_r_set(2);
    pub_com_pos_set_odom_.publish(pose_set_odom);
}

void InfoToROS::publish_pose_set_com_offseted(DogState &state)
{
    nav_msgs::Odometry pose_set_odom;
    pose_set_odom.header.frame_id = "world";
    pose_set_odom.header.stamp = ros::Time::now();
    pose_set_odom.child_frame_id = "com_set";
    pose_set_odom.pose.pose.position.x = state.ctrl.com_set(0);
    pose_set_odom.pose.pose.position.y = state.ctrl.com_set(1);
    pose_set_odom.pose.pose.position.z = state.ctrl.com_set(2);
    Eigen::Quaterniond quaternion_pos_set = Eigen::AngleAxisd(state.ctrl.com_euler_set(2) + state.ctrl.yaw_offset_set, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state.ctrl.com_euler_set(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(state.ctrl.com_euler_set(0), Eigen::Vector3d::UnitX());
    pose_set_odom.pose.pose.orientation.x = quaternion_pos_set.x();
    pose_set_odom.pose.pose.orientation.y = quaternion_pos_set.y();
    pose_set_odom.pose.pose.orientation.z = quaternion_pos_set.z();
    pose_set_odom.pose.pose.orientation.w = quaternion_pos_set.w();
    // make sure com_lin_vel is in world frame
    pose_set_odom.twist.twist.linear.x = state.ctrl.com_lin_vel_r_set(0);
    pose_set_odom.twist.twist.linear.y = state.ctrl.com_lin_vel_r_set(1);
    pose_set_odom.twist.twist.linear.z = state.ctrl.com_lin_vel_r_set(2);

    pose_set_odom.twist.twist.angular.x = state.ctrl.com_ang_vel_r_set(0);
    pose_set_odom.twist.twist.angular.y = state.ctrl.com_ang_vel_r_set(1);
    pose_set_odom.twist.twist.angular.z = state.ctrl.com_ang_vel_r_set(2);
    pub_com_pos_set_com_offseted_odom_.publish(pose_set_odom);
}

void InfoToROS::publish_base_tf(DogState &state)
{
    tf::Transform transform_base;
    transform_base.setOrigin(tf::Vector3(state.attitude.com_pos(0), state.attitude.com_pos(1), state.attitude.com_pos(2)));
    tf::Quaternion q_base(state.attitude.com_quat.x(), state.attitude.com_quat.y(), state.attitude.com_quat.z(), state.attitude.com_quat.w());
    transform_base.setRotation(q_base);
    odom_tf_broadcaster_.sendTransform(tf::StampedTransform(transform_base, ros::Time::now(), "world", "base"));
}

void InfoToROS::publish_pose_set_tf(DogState &state)
{
    tf::Transform transform_set;
    Eigen::Quaterniond quaternion_pos_set = Eigen::AngleAxisd(state.ctrl.com_euler_set(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state.ctrl.com_euler_set(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(state.ctrl.com_euler_set(0), Eigen::Vector3d::UnitX());
    transform_set.setOrigin(tf::Vector3(state.ctrl.com_set(0), state.ctrl.com_set(1), state.ctrl.com_set(2)));
    tf::Quaternion q_set(quaternion_pos_set.x(), quaternion_pos_set.y(), quaternion_pos_set.z(), quaternion_pos_set.w());
    transform_set.setRotation(q_set);
    odom_tf_broadcaster_.sendTransform(tf::StampedTransform(transform_set, ros::Time::now(), "world", "pos_set"));
}

void InfoToROS::publish_abs_tf(DogState &state)
{
    tf::Transform transform_abs;
    transform_abs.setOrigin(tf::Vector3(0, 0, 0));
    Eigen::Quaterniond q_abs_e(state.attitude.com_rot_mat_r_to_abs.transpose());
    tf::Quaternion q_abs(q_abs_e.x(), q_abs_e.y(), q_abs_e.z(), q_abs_e.w());
    transform_abs.setRotation(q_abs);
    odom_tf_broadcaster_.sendTransform(tf::StampedTransform(transform_abs, ros::Time::now(), "base", "abs"));
}

void InfoToROS::publish_world_tf(DogState &state)
{
    tf::Transform transform_world_to_odom;
    transform_world_to_odom.setOrigin(tf::Vector3(0, 0, 0));
    Eigen::Quaterniond q_world_e(Eigen::Matrix3d::Identity());
    tf::Quaternion q_world(q_world_e.x(), q_world_e.y(), q_world_e.z(), q_world_e.w());
    transform_world_to_odom.setRotation(q_world);
    odom_tf_broadcaster_.sendTransform(tf::StampedTransform(transform_world_to_odom, ros::Time::now(), "world", "world"));
}

void InfoToROS::publish_grf(DogState &state)
{
    std_msgs::Float64MultiArray grf_msg;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        for (int j = 0; j < 3; j++)
        {
            grf_msg.data.emplace_back(state.ctrl.foot_forces_grf_set.block<3, 1>(0, i)(j));
        }
    }
    pub_grf_.publish(grf_msg);
}

void InfoToROS::publish_debug_value(DogState &state)
{
    std_msgs::Float64MultiArray debug_msg;
    debug_msg.data = state.attitude.debug_value;
    pub_debug_values_.publish(debug_msg);
}

void InfoToROS::publish_mpc_reference(DogState &state)
{
    pub_mpc_reference_.publish(state.ctrl.mpc_reference);
}

void InfoToROS::publish_mpc_prediction(DogState &state)
{
    pub_mpc_prediction_.publish(state.ctrl.mpc_prediction);
}


void InfoToROS::publish_support_polygon(DogState &state)
{
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = "world";
    polygon.header.stamp = ros::Time::now();
    int index[4] = {0, 1, 3, 2};
    for(int i = 0; i < NUM_LEG; ++i)
    {
        geometry_msgs::Point32 point;
        if(state.attitude.support_polygon(0, index[i]) != 0 && state.attitude.support_polygon(1, index[i]) != 0)
        {
            point.x = state.attitude.support_polygon(0, index[i]);
            point.y = state.attitude.support_polygon(1, index[i]);
            point.z = state.attitude.support_polygon(2, index[i]);
            polygon.polygon.points.emplace_back(point);
        }
    }
    pub_support_polygon_.publish(polygon);   
}

void InfoToROS::publish_virtual_support_polygon(DogState &state)
{
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = "world";
    polygon.header.stamp = ros::Time::now();
    int index[4] = {0, 1, 3, 2};
    for(int i = 0; i < NUM_LEG; ++i)
    {
        geometry_msgs::Point32 point;
        if(state.attitude.virtual_support_polygon(0, index[i]) != 0 && state.attitude.virtual_support_polygon(1, index[i]) != 0)
        {
            point.x = state.attitude.virtual_support_polygon(0, index[i]);
            point.y = state.attitude.virtual_support_polygon(1, index[i]);
            point.z = state.attitude.virtual_support_polygon(2, index[i]);
            polygon.polygon.points.emplace_back(point);
        }
    }
    pub_virtual_support_polygon_.publish(polygon);   
}

void InfoToROS::print_motor_torque(DogState &state)
{
    std::cout << "===== motor torque info =====" << std::endl;
    for(int i = 0; i < 4; ++i)
    {
        std::cout << state.ctrl.joint_torques_set.block<3, 1>(3 * i, 0).transpose() << std::endl;
    }

}



void InfoToROS::publish_state(DogState &state)
{
    publish_joint_angle(state);
    // publish_imu_raw(state);
    // publish_com_acc_r(state);
    publish_marker(state);
    // publish_estimated_pose(state);
    publish_pose_set(state);
    publish_pose_set_com_offseted(state);
    publish_base_tf(state);
    // publish_pose_set_tf(state);
    // publish_abs_tf(state);
    // publish_world_tf(state);
    publish_grf(state);
    publish_debug_value(state);
    publish_mpc_reference(state);
    publish_mpc_prediction(state);
    // print_motor_torque(state);
    // publish_support_polygon(state);
    // publish_virtual_support_polygon(state);
}