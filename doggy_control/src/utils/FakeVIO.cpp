#include "FakeVIO.h"

FakeVIO::FakeVIO(Eigen::Vector3d vio_offset) : vio_offset_(vio_offset)
{
    ros::NodeHandle nh;
    vio_odom_ = nh.subscribe("/body_pose_ground_truth", 1, &FakeVIO::body_ground_truth_callback, this);
    pub_fake_vio_ = nh.advertise<nav_msgs::Odometry>("/camera/pose", 1);
}

void FakeVIO::body_ground_truth_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    Eigen::Quaterniond q(odom->pose.pose.orientation.w,
                         odom->pose.pose.orientation.x,
                         odom->pose.pose.orientation.y,
                         odom->pose.pose.orientation.z);
    Eigen::Vector3d p(odom->pose.pose.position.x,
                      odom->pose.pose.position.y,
                      odom->pose.pose.position.z);
    Eigen::Vector3d p_dot(odom->twist.twist.linear.x,
                      odom->twist.twist.linear.y,
                      odom->twist.twist.linear.z);                  
    Eigen::Vector3d vio_p = p + q * vio_offset_;
    Eigen::Vector3d vio_p_dot = p_dot + Utils::skew(p_dot) * q * vio_offset_;
    fake_vio_odom_.header = odom->header;
    fake_vio_odom_.header.frame_id = "world";
    fake_vio_odom_.child_frame_id = "vio";
    fake_vio_odom_.pose.pose.orientation.w = q.w();
    fake_vio_odom_.pose.pose.orientation.x = q.x();
    fake_vio_odom_.pose.pose.orientation.y = q.y();
    fake_vio_odom_.pose.pose.orientation.z = q.z();

    fake_vio_odom_.pose.pose.position.x = vio_p.x();
    fake_vio_odom_.pose.pose.position.y = vio_p.y();
    fake_vio_odom_.pose.pose.position.z = vio_p.z();

    fake_vio_odom_.twist.twist.angular = odom->twist.twist.angular;

    fake_vio_odom_.twist.twist.linear.x = vio_p_dot.x();
    fake_vio_odom_.twist.twist.linear.y = vio_p_dot.y();
    fake_vio_odom_.twist.twist.linear.z = vio_p_dot.z();

    pub_fake_vio_.publish(fake_vio_odom_);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "fake_vio");
    FakeVIO fake_vio(Eigen::Vector3d(0.22, 0.0, 0.02));
    ros::spin();
    return 0;
}