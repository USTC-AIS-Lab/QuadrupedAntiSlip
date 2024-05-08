#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include "dog_params.h"
#include "dog_state.h"
#include "Utils.h"
#include <chrono>

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

class BaseController
{
public:
    BaseController()
    {
        max_angle.setZero();
        body_height_max_delta = 0;
        controller_name = "base";
    }
    virtual Eigen::Matrix<double, 3, NUM_LEG> compute_ground_reaction_force(DogState &state, double dt) = 0;
    Eigen::Vector3d get_max_angle(){return max_angle;};
    double get_body_height_max_delta(){return body_height_max_delta;};
protected:
    Eigen::Vector3d max_angle;
    std::string controller_name;
    double body_height_max_delta;
};

#endif
