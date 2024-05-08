#include "joy_to_cmd.h"

#define BT_A 0
#define BT_X 3
#define BT_Y 4
#define BT_B 1
#define BT_START 11
#define BT_LT 8
#define BT_LB 6

/********************************
 * joy to command
 * empty:
 *      start: stand up
 *      A: walk
 *      B: stop
 * LB: for high level command
 *      start: lay down
 *      A: switch gait 1 walk
 *      B: switch gait 2 pee
 *      X: switch gait 3 probe
 *      Y: switch gait 4
 * LT: for low level command
 *      A: gait cmd 1
 *      B: gait cmd 2 
 *      X: gait cmd 3 
 *      Y: gait cmd 4
********************************/





JoyToCmd::JoyToCmd()
{
    if (!(nh_.param("doggy_joy/max_vel_x", max_vel_x_, 0.1) &&
          nh_.param("doggy_joy/max_vel_y", max_vel_y_, 0.1) &&
          nh_.param("doggy_joy/max_body_roll", max_body_roll_, 0.1) &&
          nh_.param("doggy_joy/max_body_pitch", max_body_pitch_, 0.1) &&
          nh_.param("default_body_height", default_body_height_, 0.28) &&
          nh_.param("doggy_joy/max_yaw_rate", max_yaw_rate_, 0.28) &&
          nh_.param("doggy_joy/body_height_max_delta", body_height_max_delta_, 0.1)))
    {
        ROS_ERROR("joy_to_cmd_node: Fail to load param!");
    }
    reset_doggy_cmd();
    joy_sub_ = nh_.subscribe("/joy", 3000, &JoyToCmd::joy_msg_callback, this);
    pub_doggy_cmd_ = nh_.advertise<doggy_msgs::DoggyMove>("/doggy_cmd", 100);
    first_msg_ = true;
}

void JoyToCmd::reset_doggy_cmd()
{
    doggy_cmd_.vel_x = 0;
    doggy_cmd_.vel_y = 0;
    doggy_cmd_.yaw_rate = 0;
    doggy_cmd_.body_height = 0;
    doggy_cmd_.body_roll = 0;
    doggy_cmd_.body_pitch = 0;
    doggy_cmd_.cmd_code = doggy_cmd_.GAIT_EMPTY;
    doggy_cmd_.gait_code = doggy_cmd_.GAIT_EMPTY;
    doggy_cmd_.is_walking = false;
}

bool JoyToCmd::button_edge(bool press, int button)
{
    return (joy_msg_now_.buttons[button] == press && joy_msg_last_.buttons[button] == !press);
}
void JoyToCmd::joy_msg_callback(const sensor_msgs::Joy &msg)
{
    joy_msg_now_ = msg;
    if (first_msg_)
    {
        first_msg_ = false;
        joy_msg_last_ = msg;
        return;
    }
    doggy_cmd_.gait_code = doggy_cmd_.GAIT_EMPTY;
    doggy_cmd_.cmd_code = doggy_cmd_.CMD_EMPTY;

    if (msg.buttons[BT_LB]) // LB CMD
    {
        if (msg.buttons[BT_A]) // BT_A
        {
            doggy_cmd_.cmd_code = doggy_cmd_.CMD_GAIT_SWITCH_1;
        }
        if (msg.buttons[BT_B]) // BT_B
        {
            doggy_cmd_.cmd_code = doggy_cmd_.CMD_GAIT_SWITCH_2;
        }
        if (msg.buttons[BT_X]) // BT_X
        {
            doggy_cmd_.cmd_code = doggy_cmd_.CMD_GAIT_SWITCH_3;
        }
        if (msg.buttons[BT_Y]) // BT_Y
        {
            doggy_cmd_.cmd_code = doggy_cmd_.CMD_GAIT_SWITCH_4;
        }
        if (msg.buttons[BT_START]) // start
        {
            doggy_cmd_.cmd_code = doggy_cmd_.CMD_LAY_DOWN;
            doggy_cmd_.is_walking = false;
        }
    }
    else if (msg.buttons[BT_LT]) // LT GAIT
    {
        if (msg.buttons[BT_A]) // BT_A
        {
            doggy_cmd_.gait_code = doggy_cmd_.GAIT_MOVE_1;
        }
        if (msg.buttons[BT_B]) // BT_B
        {
            doggy_cmd_.gait_code = doggy_cmd_.GAIT_MOVE_2;
        }
        if (msg.buttons[BT_X]) // BT_X
        {
            doggy_cmd_.gait_code = doggy_cmd_.GAIT_MOVE_3;
        }
        if (msg.buttons[BT_Y]) // BT_Y
        {
            doggy_cmd_.gait_code = doggy_cmd_.GAIT_MOVE_4;
        }
        
    }
    else
    {
        if (msg.buttons[BT_START]) // start
        {
            doggy_cmd_.cmd_code = doggy_cmd_.CMD_STAND_UP;
            doggy_cmd_.is_walking = false;
        }
        if (msg.buttons[BT_A])
        {
            doggy_cmd_.is_walking = true;
        }
        if (msg.buttons[BT_B])
        {
            doggy_cmd_.is_walking = false;
        }
        if (doggy_cmd_.is_walking)
        {
            doggy_cmd_.vel_y = msg.axes[0] * max_vel_y_;
            doggy_cmd_.vel_x = msg.axes[1] * max_vel_x_;
            doggy_cmd_.yaw_rate = msg.axes[2] * max_yaw_rate_;
            doggy_cmd_.body_height = 0;
        }
        else
        {
            doggy_cmd_.body_roll = -msg.axes[0] * max_body_roll_;
            doggy_cmd_.body_pitch = msg.axes[1] * max_body_pitch_;
            doggy_cmd_.body_yaw = msg.axes[2];
            doggy_cmd_.yaw_rate = 0;
            doggy_cmd_.body_height = msg.axes[3] * body_height_max_delta_;
        }
    }

    doggy_cmd_.proportional_cmd = true;
    doggy_cmd_.proportional_attitude = true;
    doggy_cmd_.reverve_val1 = (0.5 - msg.axes[5] * 0.5);
    doggy_cmd_.reverve_val2 = (0.5 - msg.axes[4] * 0.5);
    joy_msg_last_ = msg;
}

void JoyToCmd::publish_doggy_cmd()
{
    pub_doggy_cmd_.publish(doggy_cmd_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_cmd_node");
    std::unique_ptr<JoyToCmd> a1 = std::make_unique<JoyToCmd>();
    ros::Rate rate(1000);
    while (ros::ok())
    {
        a1->publish_doggy_cmd();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}