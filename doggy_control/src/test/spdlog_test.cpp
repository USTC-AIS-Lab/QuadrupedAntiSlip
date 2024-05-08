#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "Utils.h"
#include <Eigen/Dense>
#include "spdlog/sinks/stdout_color_sinks.h"


int main(int argc, char **argv)
{
    std::string file_name = Utils::getCurrentTimeAsString() + ".txt";
    auto my_logger = spdlog::basic_logger_mt("file_logger", "/root/A1_ctrl_ws/src/dog_ctrl/doggy_control/logs/" + file_name);
    //Use the default logger (stdout, multi-threaded, colored)
    my_logger->info("Hello, {}!", "World");
    spdlog::info("Hello, {}!", "World");
    spdlog::warn("Warn, {}!", 1.0);

    Eigen::Vector3d vec = Eigen::Vector3d::Random();
    std::ostringstream buffer;
    buffer << vec.transpose();
    std::string str_vec = buffer.str();
    
    my_logger->info(str_vec);
}

