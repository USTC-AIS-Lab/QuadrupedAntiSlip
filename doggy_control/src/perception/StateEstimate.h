#ifndef STATE_ESTIMATE_H_
#define STATE_ESTIMATE_H_

#include "dog_state.h"
#include "utils/Utils.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <nav_msgs/Odometry.h>
#define STATE_SIZE 18
#define MEASURE_SIZE 30 // (foot position residual:4*3) (foot velocity residual:4*3) (foot height:4) (vioxy:2)

#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.01 //0.01
#define SENSOR_NOISE_PIMU_REL_FOOT 1.0
#define SENSOR_NOISE_VIMU_REL_FOOT 100.0//0.1
#define SENSOR_NOISE_ZFOOT 0.1

#define INITIAL_HEIGHT 0.09

#define CONTACT_NOISE_RATIO 5e3

#define SLIP_NOISE_RATIO 5e2

class StateEstimate
{
public:
    StateEstimate();
    void init_state(DogState& state);
    void update_estimation(DogState& state, double dt);
    bool is_inited() {return filter_initialized;}
private:
    bool filter_initialized = false;
    Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
    // state
    // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
    Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state
    Eigen::Matrix<double, STATE_SIZE, 1> xbar; // estimation state after process update

    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, 3> B; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> R; // estimation state transition noise
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance sigma
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update

    Eigen::Matrix<double, MEASURE_SIZE, STATE_SIZE> C; // estimation state observation
    Eigen::Matrix<double, MEASURE_SIZE, MEASURE_SIZE> Q; // estimation state observation noise
    // observation
    // 0 1 2   FL pos residual
    // 3 4 5   FR pos residual
    // 6 7 8   RL pos residual
    // 9 10 11 RR pos residual
    // 12 13 14 vel residual from FL
    // 15 16 17 vel residual from FR
    // 18 19 20 vel residual from RL
    // 21 22 23 vel residual from RR
    // 24 25 26 27 foot height
    Eigen::Matrix<double, MEASURE_SIZE, 1> z; //  observation
    Eigen::Matrix<double, MEASURE_SIZE, 1> zhat; // estimated observation
    Eigen::Matrix<double, MEASURE_SIZE, MEASURE_SIZE> S; // Innovation (or pre-fit residual) covariance
    double vio_noise;



    
};

#endif