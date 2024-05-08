#include "vio_udp.h"

VIO_UDP::VIO_UDP() : udp(LOCAL_PORT, TARGET_IP, TARGET_PORT, sizeof(VIOInfo), sizeof(VIOInfo))
{
    vio_noise = 0.0;
    dt = 0.05;
    VIO_info.position.x = 0;
    VIO_info.position.y = 0;
    VIO_info.position.z = 0;
    VIO_info.orientation.x = 0;
    VIO_info.orientation.y = 0;
    VIO_info.orientation.z = 0;
    VIO_info.orientation.w = 1;
    VIO_info.seq = 1;
    rot_mat_world_to_cam_ << 1, 0, 0,
                             0, 0, -1,
                             0, 1, 0;
    rot_mat_pose_ << 0, 1, 0,
                    -1, 0, 0,
                     0, 0, 1;
    camera_offset_ << 0.26, 0.0, 0.04;
}

Eigen::Vector3d VIO_UDP::get_position(void)
{

    return rot_mat_pose_ * Eigen::Vector3d(VIO_info.position.x, VIO_info.position.y, VIO_info.position.z) - get_orientation().inverse() * camera_offset_;
}

Eigen::Quaterniond VIO_UDP::get_orientation()
{
    return Eigen::Quaterniond(rot_mat_world_to_cam_) * Eigen::Quaterniond(VIO_info.orientation.w, VIO_info.orientation.x, VIO_info.orientation.y, VIO_info.orientation.z);
}

void VIO_UDP::UDPRecv()
{
    udp.Recv();
    udp.GetRecv((char *)&VIO_info);
}

void VIO_UDP::UDPSend()
{
    udp.SetSend((char*)&VIO_info);
    udp.Send();
}

void VIO_UDP::set_position(Eigen::Vector3d postion)
{
    VIO_info.position.x = postion.x();
    VIO_info.position.y = postion.y();
    VIO_info.position.z = postion.z();
}

void VIO_UDP::set_orientation(Eigen::Quaterniond orientation)
{
    VIO_info.orientation.x = orientation.x();
    VIO_info.orientation.y = orientation.y();
    VIO_info.orientation.z = orientation.z();
    VIO_info.orientation.w = orientation.w();
}

void VIO_UDP::send_vio(Eigen::Vector3d postion, Eigen::Quaterniond orientation)
{
    set_position(postion);
    set_orientation(orientation);
    VIO_info.seq ++;
    UDPSend();
}