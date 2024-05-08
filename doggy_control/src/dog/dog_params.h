#ifndef DOGPARAMS_H
#define DOGPARAMS_H

// FL, FR, RL, RR
// control time related
#define GRF_UPDATE_INTERVEL_MS 1.6 // ms 
#define MAIN_UPDATE_INTERVEL_MS 1.0 // ms
#define HARDWARE_FEEDBACK_INTERVEL_MS 0.8  // ms

#define SAFE_POWER_LEVEL 9
#define MAX_TORQUE 0.0
#define TILT_THRESHOLD 0.3
#define GRAVITY_ACCELERATION 9.81

#define ROBOT_MASS 14.0
#define TRUNK_INERTIA_X 0.0158533
#define TRUNK_INERTIA_Y 0.0377999
#define TRUNK_INERTIA_Z 0.0456542

#define FRITION_MU 0.7

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

#define DEFAULT_FOOT_POS_X_ABS 0.17
#define DEFAULT_FOOT_POS_Y_ABS 0.15

#define DEFAULT_BODY_HEIGHT 0.25

#define GRAVITY_COMPENSATION 0.0 // 0.80

#define FOOT_FORCE_FILTER 0.95
#define FOOT_CONTACT_THRESHOLD 50.0

#define KP_FOOT_X 200.0
#define KP_FOOT_Y 200.0
#define KP_FOOT_Z 250.0

#define KD_FOOT_X 5.0
#define KD_FOOT_Y 5.0
#define KD_FOOT_Z 5.0

#define KP_ANGULAR_X 650.0
#define KP_ANGULAR_Y 35.0
#define KP_ANGULAR_Z 0.0 //1.0

#define KD_ANGULAR_X 4.5
#define KD_ANGULAR_Y 4.5
#define KD_ANGULAR_Z 0.0 //30.0

#define KP_LINEAR_X 100.0
#define KP_LINEAR_Y 100.0
#define KP_LINEAR_Z 100.0

#define KD_LINEAR_X 20.0
#define KD_LINEAR_Y 7.0
#define KD_LINEAR_Z 12.0

#define QP_ALPHA 1e-3
#define QP_BETA 1e-4
#define QP_S_POS_X 1.0
#define QP_S_POS_Y 1.0
#define QP_S_POS_Z 1.0
#define QP_S_ANG_X 400.0
#define QP_S_ANG_Y 400.0
#define QP_S_ANG_Z 100.0

#define CONSTRAINT_F_MAX 180.0
#define CONSTRAINT_F_MIN 0.0



#define PRONE_THRESHOLD_Z -0.08
#endif //A1_CPP_A1PARAMS_H
