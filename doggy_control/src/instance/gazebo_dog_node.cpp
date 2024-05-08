#include "gazebo_dog.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebodog_ctrl_node");
    ros::NodeHandle nh;
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    bool use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time)) {
        if (use_sim_time != true) {
            std::cout << "simulation must have simulated time in order to use this program!" << std::endl;
            return -1;
        }
    }
    std::unique_ptr<GazeboDog> dog = std::make_unique<GazeboDog>(nh);

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);
    std::thread compute_foot_forces_grf_thread([&]()
    {
        while (control_execute.load(std::memory_order_acquire) && ros::ok())
        {
            ros::Time now = ros::Time::now();
            auto t1 = std::chrono::high_resolution_clock::now();
            dog->update_foot_forces_grf(GRF_UPDATE_INTERVEL_MS * 1e-3);
            ros::Duration dt_solver_time = ros::Time::now() - now;
            auto t2 = std::chrono::high_resolution_clock::now();
            //std::cout << "update_foot_forces_grf time: " << ((std::chrono::duration<double, std::milli>)(t2 - t1)).count() << "ms" << std::endl;
            if (dt_solver_time.toSec() < GRF_UPDATE_INTERVEL_MS / 1000.0) 
            {
                ros::Duration( GRF_UPDATE_INTERVEL_MS / 1000 - dt_solver_time.toSec() ).sleep();
            }
            // else
            // {
            //     std::cout << "compute_foot_forces_grf_thread time:" << dt_solver_time.toSec() * 1000.0 << ", ";
            //     std::cout << ((std::chrono::duration<double, std::milli>)(t2 - t1)).count() << "ms" << std::endl;
            // }
        }
    });
    std::thread main_thread([&]() 
    {
        while (control_execute.load(std::memory_order_acquire) && ros::ok())
        {
            ros::Time now = ros::Time::now();
            // auto t1 = std::chrono::high_resolution_clock::now();
            bool main_update_running = dog->main_update(MAIN_UPDATE_INTERVEL_MS * 0.001);
            bool send_cmd_running = dog->send_cmd();
            auto t2 = std::chrono::high_resolution_clock::now();
            // std::cout << "main_update_running time: " << ((std::chrono::duration<double, std::milli>)(t2 - t1)).count() << "ms" << std::endl;
            ros::Duration dt_solver_time = ros::Time::now() - now;

            if (dt_solver_time.toSec() < MAIN_UPDATE_INTERVEL_MS / 1000.0) 
            {
                ros::Duration( MAIN_UPDATE_INTERVEL_MS / 1000 - dt_solver_time.toSec()).sleep();
            }
            // else
            // {
            //     std::cout << "main_thread time:" << dt_solver_time.toSec() * 1000.0 << ", ";
            //     std::cout << ((std::chrono::duration<double, std::milli>)(t2 - t1)).count() << "ms" << std::endl;
            // }
        }
    });
    ros::AsyncSpinner spinner(12);
    spinner.start();
    compute_foot_forces_grf_thread.join();
    main_thread.join();
    return 0;
}