#include "StateEstimate.h"
#include <chrono>
// state x
// 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR

//control u = com_acc
//Ra + g

// observation z
// 0 1 2   FL pos residual
// 3 4 5   FR pos residual
// 6 7 8   RL pos residual
// 9 10 11 RR pos residual
// 12 13 14 vel residual from FL
// 15 16 17 vel residual from FR
// 18 19 20 vel residual from RL
// 21 22 23 vel residual from RR
// 24 25 26 27 foot height



StateEstimate::StateEstimate()
{
    eye3.setIdentity();
    C.setZero();
    vio_noise = 1e10;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        //foot pos residual = foot pos - pos
        C.block<3, 3>(i * 3, 0) = -eye3;        //-pos
        C.block<3, 3>(i * 3, 6 + i * 3) = eye3; //foot pos
        //foot vel residual = vel ?? #TODO should the vel of foot be taken into account?
        C.block<3, 3>(NUM_LEG * 3 + i * 3, 3) = eye3; // vel
        //foot height (NOT pose) = z of foot pose
        C(NUM_LEG * 6 + i, 6 + i * 3 + 2) = 1; //height z of foot
    }
    C.block<2, 2>(28, 0) = Eigen::Matrix2d::Identity();
    // set A to identity
    A.setIdentity();

    // set B to zero
    B.setZero();

    // Q R are fixed
    R.setIdentity();
    // R.block<3,3>(0,0) = PROCESS_NOISE_PIMU * eye3;               // position transition
    // R.block<3,3>(3,3) = PROCESS_NOISE_VIMU * eye3;               // velocity transition
    for (int i = 0; i < NUM_LEG; ++i)
    {
        R.block<3, 3>(6 + i * 3, 6 + i * 3) = PROCESS_NOISE_PFOOT * eye3; // foot position transition
    }

    Q.setIdentity();
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Q.block<3, 3>(i * 3, i * 3) = SENSOR_NOISE_PIMU_REL_FOOT * eye3;                             // fk estimation
        Q.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = SENSOR_NOISE_VIMU_REL_FOOT * eye3; // vel estimation
        Q(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = SENSOR_NOISE_ZFOOT;                                    // height z estimation
    }
    Q.block<2, 2>(28, 28) = vio_noise * Eigen::Matrix2d::Identity();
}

void StateEstimate::init_state(DogState &state)
{
    P.setIdentity();
    P = P * 3;
    // set initial value of x
    x.setZero();
    x.segment<3>(0) = Eigen::Vector3d(0, 0, INITIAL_HEIGHT);
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Vector3d fk_pos = state.attitude.foot_pos_r.block<3, 1>(0, i);
        // init foot pose
        x.segment<3>(6 + i * 3) = state.attitude.com_rot_mat * fk_pos + x.segment<3>(0);
    }
    filter_initialized = true;
}

void StateEstimate::update_estimation(DogState &state, double dt)
{
    //std::cout << "****** start time counting. ******" << std::endl;
    


    
    /*************** contact_estimate **************/
    /*************** update A B using latest dt**************/
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;

    // control input u is Ra + ag
    Eigen::Vector3d u = state.attitude.com_rot_mat * state.attitude.com_acc_r;

    /*********************update Q R***********************/
    //pos noice
    R.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt * eye3;
    //vel noice
    R.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * eye3;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        //foot pose noise in transition
        R.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - state.attitude.is_contact[i]) * CONTACT_NOISE_RATIO + state.slip_info.slip_state(i) * SLIP_NOISE_RATIO) * PROCESS_NOISE_PFOOT * dt * eye3;
        // foot position transition
        // for estimated_contacts[i] == 1, R = 0.002
        // for estimated_contacts[i] == 0, R = 1001*R

        //foot pose noise in measurement
        Q.block<3, 3>(i * 3, i * 3) = (1 + (1 - state.attitude.is_contact[i]) * CONTACT_NOISE_RATIO) * SENSOR_NOISE_PIMU_REL_FOOT * dt * eye3;
        //foot vel noise in measurement
        Q.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - state.attitude.is_contact[i]) * CONTACT_NOISE_RATIO) * SENSOR_NOISE_VIMU_REL_FOOT * dt * eye3;
        //foot height noise in measurement
        Q(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - state.attitude.is_contact[i]) * CONTACT_NOISE_RATIO) * SENSOR_NOISE_ZFOOT * dt;
    }
    Q.block<2, 2>(28, 28) = vio_noise * Eigen::Matrix2d::Identity();


    /****************pridiction step***************************/
    xbar = A * x + B * u;
    Pbar = A * P * A.transpose() + R;
    zhat = C * xbar;

    /****************update step***************************/
    //take measurement
    for (int i = 0; i < NUM_LEG; ++i)
    {
        Eigen::Vector3d fk_pos = state.attitude.foot_pos_r.block<3, 1>(0, i);
        //measure foot pose residual
        z.block<3, 1>(i * 3, 0) = state.attitude.com_rot_mat * fk_pos; // fk estimation
        Eigen::Vector3d leg_v = -state.attitude.foot_vel_r.block<3, 1>(0, i) - Utils::skew(state.attitude.com_ang_vel_r) * fk_pos;
        //measure foot vel residual
        z.block<3, 1>(NUM_LEG * 3 + i * 3, 0) =
            (1.0 - state.attitude.is_contact[i]) * x.segment<3>(3) + state.attitude.is_contact[i] * state.attitude.com_rot_mat * leg_v; // vel estimation
        //measure foot height
        z(NUM_LEG * 6 + i) =
            (1.0 - state.attitude.is_contact[i]) * (x(2) + (state.attitude.com_rot_mat * fk_pos)(2)) + state.attitude.is_contact[i] * 0; // height z estimation
    }
    z.segment<2>(28) = Eigen::Vector2d(state.attitude.pos_vio(0), state.attitude.pos_vio(1));
    // std::cout << state.attitude.pos_vio.transpose() << std::endl;

    // auto end = std::chrono::system_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "take measurement: " << duration.count() << " ms\n";
    // start = end;

    S = C * Pbar * C.transpose() + Q;
    S = 0.5*(S+S.transpose()); // what's this for? make sure it keep symmetrical

    Eigen::Matrix<double, MEASURE_SIZE, MEASURE_SIZE> S_inverse = S.inverse();
    //x = xbar + Pbar * C.transpose() * S.fullPivHouseholderQr().solve(z - zhat);
    x = xbar + Pbar * C.transpose() * S_inverse * (z - zhat);
    x(2) = std::max(x(2), 0.06); // robot cannot be that low.
    //P = Pbar - Pbar * C.transpose() * S.fullPivHouseholderQr().solve(C) * Pbar;
    P = Pbar - Pbar * C.transpose() * S_inverse * C * Pbar;
    P = 0.5 * (P + P.transpose());

    // end = std::chrono::system_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "get x: " << duration.count() << " ms\n";
    // start = end;


    // reduce position drift
    if (P.block<2, 2>(0, 0).determinant() > 1e-6) 
    {
        P.block<2, 16>(0, 2).setZero();
        P.block<16, 2>(2, 0).setZero();
        P.block<2, 2>(0, 0) /= 10.0;
    }
    // std::cout << "P.block<2, 2>(0, 0)" << std::endl;
    // std::cout << P.block<2, 2>(0, 0) << std::endl;

    state.attitude.com_pos_estimated = x.segment<3>(0);
    state.attitude.com_lin_vel_estimated = x.segment<3>(3);
    state.attitude.com_pos = state.attitude.com_pos_estimated;
    state.attitude.com_lin_vel = state.attitude.com_lin_vel_estimated;
    state.attitude.com_lin_vel_r = state.attitude.com_rot_mat.transpose() * state.attitude.com_lin_vel;

    //state.attitude.com_pos = state.ctrl.com_pos_set;
    //state.attitude.com_pos.setZero();
    // state.attitude.com_pos(2) = - (state.attitude.foot_pos_r(2,0) + state.attitude.foot_pos_r(2,1) + state.attitude.foot_pos_r(2,2) + state.attitude.foot_pos_r(2,3)) / 4.0;
    //state.attitude.com_lin_vel.setZero();

    // std::cout << state.attitude.foot_force.transpose() << std::endl;
    // std::cout << "state.attitude.com_acc_r: " << std::endl;
    // std::cout << state.attitude.com_acc_r.transpose() << std::endl;

    // std::cout << "u: " << std::endl;
    // std::cout << u.transpose() << std::endl;
    
    // std::cout << "x: " << std::endl;
    // std::cout << x.transpose() << std::endl;
    
    // std::cout << "z: " << std::endl;
    // std::cout << z.transpose() << std::endl;

    // state.attitude.estimated_state.com_pos = x.segment<3>(0);
    // state.attitude.estimated_state.root_var = P.block<3, 3>(0, 0);
    // for(int i = 0; i < NUM_LEG; ++i)
    // {
    //     state.attitude.estimated_state.foot_pos.block<3, 1>(0, i) = x.segment<3>(6 + 3 * i);
    //     state.attitude.estimated_state.foot_var.block<3, 3>(i * 3, i * 3) = P.block<3, 3>(i * 3 + 6, i * 3 + 6);
    // }

    // std::cout << "P:" << std::endl;
    // std::cout << P.block<3, 3>(0, 0) << std::endl;
    // std::cout << "Pbar:" << std::endl;
    // std::cout << Pbar.block<3, 3>(0, 0) << std::endl;

    // state.attitude.debug_value[0] = u(0);
    // state.attitude.debug_value[1] = u(1);
    // state.attitude.debug_value[2] = u(2);

}


