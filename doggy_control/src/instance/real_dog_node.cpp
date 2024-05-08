#include "real_dog.cpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "realdog_ctrl_node");
    ros::NodeHandle nh;

    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }
    bool use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time)) {
        if (use_sim_time != false) {
            std::cout << "hardware must have real time in order to use this program!" << std::endl;
            return -1;
        }
    }
    std::unique_ptr<RealDog> dog = std::make_unique<RealDog>(nh);
    dog->wait_for_udp_received();

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);
    std::thread compute_foot_forces_grf_thread([&]()
    {
        while (control_execute.load(std::memory_order_acquire) && ros::ok())
        //while (ros::ok())
        {
            auto t1 = std::chrono::high_resolution_clock::now();
            dog->update_foot_forces_grf(GRF_UPDATE_INTERVEL_MS * 0.001);
            auto t2 = std::chrono::high_resolution_clock::now();
            int dt_us = ((std::chrono::duration<double, std::micro>)(t2 - t1)).count();
            if (dt_us < GRF_UPDATE_INTERVEL_MS * 1e3) 
            {
                std::this_thread::sleep_for(std::chrono::microseconds((int)(GRF_UPDATE_INTERVEL_MS * 1e3 - dt_us)));
            }
            else if (dt_us > 2 * GRF_UPDATE_INTERVEL_MS * 1e3) 
            {
                // dog->log(spdlog::level::warn, "grf solve time = {} ms, over {} ms", dt_us * 1e-3, GRF_UPDATE_INTERVEL_MS);
                // std::cout << "grf solve time = " << dt_us * 1e-3 << "ms, over " << GRF_UPDATE_INTERVEL_MS << std::endl;
            }
            // dog->log(spdlog::level::warn, "grf solve time = {} ms, over {} ms", dt_us * 1e-3, GRF_UPDATE_INTERVEL_MS);
        }
    });
    std::thread main_thread([&]() 
    {

        while (control_execute.load(std::memory_order_acquire) && ros::ok())
        //while (ros::ok())
        {
            auto t1 = std::chrono::high_resolution_clock::now();
            bool main_update_running = dog->main_update(MAIN_UPDATE_INTERVEL_MS * 0.001);
            bool send_cmd_running = dog->send_cmd();

            if (dog->safety_halt()) 
            {
                ros::shutdown();
                std::terminate();
                break;
            }

            
            auto t2 = std::chrono::high_resolution_clock::now();
            int dt_us = ((std::chrono::duration<double, std::micro>)(t2 - t1)).count();
            if (dt_us < MAIN_UPDATE_INTERVEL_MS * 1e3) 
            {
                std::this_thread::sleep_for(std::chrono::microseconds((int)(MAIN_UPDATE_INTERVEL_MS * 1e3 - dt_us)));
            }
            else
            {
                dog->log(spdlog::level::warn, "main solve time = {} ms, over {} ms", dt_us * 1e-3, MAIN_UPDATE_INTERVEL_MS);
                //std::cout << "main solve time = " << dt_us * 1e-3 << "ms, over " << MAIN_UPDATE_INTERVEL_MS << std::endl;
            }
            // dog->log(spdlog::level::warn, "main solve time = {} ms, over {} ms", dt_us * 1e-3, MAIN_UPDATE_INTERVEL_MS);
        }
    });
    ros::AsyncSpinner spinner(12);
    spinner.start();
    compute_foot_forces_grf_thread.join();
    main_thread.join();
    return 0;
}