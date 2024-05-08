#ifndef _SLIP_DETECTION_H_
#define _SLIP_DETECTION_H_


#include "dog_params.h"
#include "dog_state.h"
#include "gait/base_gait.h"

typedef std::unique_ptr<BaseGait> GaitPtr;

class SlipDetector
{
public:
    SlipDetector(GaitPtr &gait_ptr);
    void detect(DogState &state);
    Eigen::Vector4i get_slip_state(){return slip_state;}
    Eigen::Vector4d get_slip_score(){return slip_score;}
    Eigen::Matrix<double, 3, NUM_LEG> get_foot_slip_spot(){return foot_slip_spot;};

private:
    GaitPtr &gait_ptr_;
    Eigen::Vector4d foot_lateral_vel_norm;
    Eigen::Vector4d foot_lateral_dis_norm;
    Eigen::Vector4i slip_state;
    Eigen::Vector4i last_contact_state;
    Eigen::Vector4d slip_score;
    Eigen::Vector4d slip_score_filtered;
    Eigen::Matrix<double, 3, NUM_LEG> foot_slip_spot;
    Eigen::Matrix<double, 3, NUM_LEG> foot_first_contact_spot;
    double detect_kp;
    double detect_kd;
    double lateral_vel_filter;
};



#endif
