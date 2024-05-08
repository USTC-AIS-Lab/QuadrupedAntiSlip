#ifndef _JOY_TO_CMD_H_
#define _JOY_TO_CMD_H_

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>

#include "doggy_msgs/DoggyMove.h"
#include "sensor_msgs/Joy.h"

class JoyToCmd
{
public:
    JoyToCmd();
    void publish_doggy_cmd();
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_doggy_cmd_;
    ros::Subscriber joy_sub_;
    doggy_msgs::DoggyMove doggy_cmd_;
    sensor_msgs::Joy joy_msg_now_;
    sensor_msgs::Joy joy_msg_last_;
    void reset_doggy_cmd();
    bool button_edge(bool press, int button);
    void joy_msg_callback(const sensor_msgs::Joy &msg);
    bool first_msg_;
    double max_vel_x_;
    double max_vel_y_;
    double max_yaw_rate_;
    double max_body_roll_;
    double max_body_pitch_;
    double default_body_height_;
    double body_height_max_delta_;
};

#endif
