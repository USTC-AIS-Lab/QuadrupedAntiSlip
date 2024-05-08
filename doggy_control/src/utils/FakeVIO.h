#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Utils.h>
#include "dog_state.h"

class FakeVIO
{
public:
    FakeVIO(Eigen::Vector3d vio_offset);
    
private:
    ros::Subscriber vio_odom_;
    ros::Publisher pub_fake_vio_;
    Eigen::Vector3d vio_offset_;
    nav_msgs::Odometry fake_vio_odom_;

    void body_ground_truth_callback(const nav_msgs::Odometry::ConstPtr &odom);
};