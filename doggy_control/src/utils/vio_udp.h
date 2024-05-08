#pragma once

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <Eigen/Dense>


#define ON_NUC


#define NX_IP "192.168.123.12"
#define NUC_IP "192.168.123.198"
#define NX_PORT 8025
#define NUC_PORT 8024

#ifdef ON_NUC
    #define TARGET_IP NX_IP
    #define TARGET_PORT NX_PORT
    #define LOCAL_PORT NUC_PORT
#else
    #define TARGET_IP NUC_IP
    #define TARGET_PORT NUC_PORT
    #define LOCAL_PORT NX_PORT
#endif

struct VIOInfo
{
    struct Position
    {
        float x;
        float y;
        float z;
    } position;
    struct Orientation
    {
        float x;
        float y;
        float z;
        float w;
    } orientation;
    uint32_t seq;
    uint32_t crc;
};

class VIO_UDP
{
public:
    VIO_UDP();
    Eigen::Vector3d get_position(void);
    Eigen::Quaterniond get_orientation();
    double get_noise(void) { return vio_noise; };
    void UDPRecv();
    void UDPSend();
    double get_dt(){return dt;};
    void set_position(Eigen::Vector3d postion);
    void set_orientation(Eigen::Quaterniond orientation);
    void send_vio(Eigen::Vector3d postion, Eigen::Quaterniond orientation);
    VIOInfo get_VIO_info(){return VIO_info;}

private:
    VIOInfo VIO_info;
    UNITREE_LEGGED_SDK::UDP udp;
    double vio_noise;
    double dt;
    Eigen::Matrix3d rot_mat_world_to_cam_;
    Eigen::Matrix3d rot_mat_pose_;
    Eigen::Vector3d camera_offset_;
    
};