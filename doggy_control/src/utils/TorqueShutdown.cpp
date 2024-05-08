#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>

using namespace std;
int main(int argc, char **argv)
{
    cout<< "Program start!"<<endl;
    UNITREE_LEGGED_SDK::LowCmd cmd = {0};
    UNITREE_LEGGED_SDK::LowState state = {0};
    UNITREE_LEGGED_SDK::UDP udp(UNITREE_LEGGED_SDK::LOWLEVEL);
    UNITREE_LEGGED_SDK::Safety safe(UNITREE_LEGGED_SDK::LeggedType::A1);
    udp.InitCmdData(cmd);
    cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for (int i = 0; i < 12; i++) 
    {
        cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;        // 禁止位置环
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;        // 禁止速度环
        cmd.motorCmd[i].Kd = 0;
        cmd.motorCmd[i].tau = 0;
    }
    safe.PositionLimit(cmd);
    udp.SetSend(cmd);
    udp.Send();
    return 0;
}