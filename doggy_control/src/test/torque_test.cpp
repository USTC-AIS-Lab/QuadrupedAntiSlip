#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include "sensor_msgs/Joy.h"

#include "unitree_legged_sdk/unitree_legged_sdk.h"



double TORQUE;
int MOTOR;
double AXIS;
double FRICTION_S = 0.4;
double THRESHOLD_V = 0.10;
double BETA_V = 0.02;

class UDPLowComm
{
public:
    UDPLowComm(int _power_level) : safe(UNITREE_LEGGED_SDK::LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL), power_level(_power_level)
    {
        udp.InitCmdData(cmd);
    }
    void receive()
    {
        udp.Recv();
        udp.GetRecv(state);
    }
    void send_init();
    void send()
    {
        safe.PositionLimit(cmd);
        udp.SetSend(cmd);
        safe.PowerProtect(cmd, state, power_level);
        udp.Send();
    }
    int power_level;
    UNITREE_LEGGED_SDK::UDP udp;
    UNITREE_LEGGED_SDK::Safety safe;
    UNITREE_LEGGED_SDK::LowState state = {0};
    UNITREE_LEGGED_SDK::LowCmd cmd = {0};
};

UDPLowComm udp_comm(5);

void joy_msg_callback(const sensor_msgs::Joy &msg)
{
    AXIS = msg.axes[1];
}


void send_torque_command(int motor, double torque)
{
    for(int i = 0; i < 12; i++)
    {
        udp_comm.cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
        udp_comm.cmd.motorCmd[i].mode = 0x0A;
        if(i == motor)
        {
            udp_comm.cmd.motorCmd[i].tau = torque;
        }
        else
        {
            udp_comm.cmd.motorCmd[i].tau = 0;
        }
        udp_comm.cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // shut down position control
        udp_comm.cmd.motorCmd[i].Kp = 0;
        udp_comm.cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // shut down velocity control
        udp_comm.cmd.motorCmd[i].Kd = 0;
        udp_comm.send();
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_cmd_node");
    ros::NodeHandle nh;
    MOTOR = std::stoi(argv[argc - 1]);
    std::cout << "start torque test for motor " << MOTOR << std::endl;
    ros::Subscriber joy_sub_ = nh.subscribe("/joy", 3000, joy_msg_callback);
    UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv("udp_recv", 0.1, 2, boost::bind(&UDPLowComm::receive, &udp_comm));
    loop_udpRecv.start();
    ros::Rate rate(1000);
    int t = 0;
    double q_filtered = 0;
    while (ros::ok())
    {
        t ++;
        q_filtered = udp_comm.state.motorState[MOTOR].dq;
        ros::spinOnce();
        TORQUE = AXIS * 4.0;
        double friction_compensation = 0.0;
        if( q_filtered > THRESHOLD_V)
        {
            friction_compensation = FRICTION_S + BETA_V * q_filtered;
        }
        else if(q_filtered < -THRESHOLD_V)
        {
            friction_compensation = - FRICTION_S + BETA_V * q_filtered;
        }
        send_torque_command(MOTOR, TORQUE + friction_compensation);
        if(t % 100 == 0)
        {
            std::cout << "torque at motor " << MOTOR << " is " <<  TORQUE << std::endl;

            std::cout << q_filtered << std::endl;
        }
        rate.sleep();
        
    }

    return 0;
}





