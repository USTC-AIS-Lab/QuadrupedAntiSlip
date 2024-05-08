#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include "dog_params.h"

#include "real_dog.cpp"

int main(int argc, char **argv) {
    std::cout << COUT_RED<<"The dog is going to dance WILDLY!" << COUT_RESET<< std::endl
              << COUT_RED<<"WARNING: Make sure the robot is HUNG UP." << COUT_RESET<< std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "test_pd_ctrl");
    ros::NodeHandle nh;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    bool use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time)) 
    {
        if (use_sim_time != false) 
        {
            std::cout << "hardware must have real time in order to use this program!" << std::endl;
            return -1;
        }
    }
    std::unique_ptr<RealDog> dog = std::make_unique<RealDog>(nh);

    std::thread main_thread([&]() 
    {
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        int time_step = 0;
        int T = 120;
        double height_min = -0.3;
        double height_max = -0.1;
        while (ros::ok())
        {
            ros::Time now = ros::Time::now();
            ros::Duration dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;
            bool main_update_running = dog->main_update(elapsed.toSec(), dt.toSec());
            bool send_cmd_running = dog->send_cmd();
            time_step ++;
            if(argc == 2 && strcmp(argv[1], "1") == 0)
            {
                dog->state_.attitude.hip_pos_r(2, 0) = 0.5 * std::sin(double(time_step + T * 0.5) / double(T) * 3.1415) * (height_max - height_min) + height_min + (height_max - height_min) * 0.5;
                dog->state_.attitude.hip_pos_r(2, 1) = 0.5 * std::sin(double(time_step + T * 1.0) / double(T) * 3.1415) * (height_max - height_min) + height_min + (height_max - height_min) * 0.5;
                dog->state_.attitude.hip_pos_r(2, 2) = 0.5 * std::sin(double(time_step + T * 1.5) / double(T) * 3.1415) * (height_max - height_min) + height_min + (height_max - height_min) * 0.5;
                dog->state_.attitude.hip_pos_r(2, 3) = 0.5 * std::sin(double(time_step + T * 2.0) / double(T) * 3.1415) * (height_max - height_min) + height_min + (height_max - height_min) * 0.5;
            }
            else
            {
                dog->state_.attitude.hip_pos_r(2, 0) = 0.5 * std::sin(double(time_step + T * 0.5) / double(T) * 3.1415) * (height_max - height_min) + height_min + (height_max - height_min) * 0.5;
                dog->state_.attitude.hip_pos_r(2, 1) = dog->state_.attitude.hip_pos_r(2, 0); 
                dog->state_.attitude.hip_pos_r(2, 2) = dog->state_.attitude.hip_pos_r(2, 0);
                dog->state_.attitude.hip_pos_r(2, 3) = dog->state_.attitude.hip_pos_r(2, 0);
            }
            
            std::cout<<"current height on leg FL: " << dog->state_.attitude.hip_pos_r(2, 0) <<std::endl;
            
            ros::Duration dt_solver_time = ros::Time::now() - now;
            if (dt_solver_time.toSec() < MAIN_UPDATE_INTERVEL_MS / 1000.0) 
            {
                ros::Duration( MAIN_UPDATE_INTERVEL_MS / 1000 - dt_solver_time.toSec() ).sleep();
            }
        }
    });
    ros::AsyncSpinner spinner(12);
    spinner.start();
    main_thread.join();
    ros::waitForShutdown();
    return 0;
}