#ifndef _ROBOT_LOG_H_
#define _ROBOT_LOG_H_

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "dog_state.h"
#include "spdlog/async.h"

#define LOG_T 5

class RobotLog
{
public:
    RobotLog();
    void init();
    void write_log(DogState &state);
    template <typename T>
    void log(spdlog::level::level_enum lvl, const T &msg)
    {
        spdlog::log(lvl, msg);
        logger_ptr_->log(lvl, msg);
    }

    template <typename... Args>
    void log(spdlog::level::level_enum lvl, spdlog::format_string_t<Args...> fmt, Args &&...args)
    {
        spdlog::log(lvl, fmt, args...);
        logger_ptr_->log(lvl, fmt, args...);
    }
    template <typename T>
    void info(const T &msg)
    {
        log(spdlog::level::info, msg);
    }
    template <typename... Args>
    void info(spdlog::format_string_t<Args...> fmt, Args &&...args)
    {
        log(spdlog::level::info, fmt, args...);
    }

private:
    int tick_;
    std::string log_file_;
    std::string log_str_;
    std::shared_ptr<spdlog::logger> logger_ptr_;
    std::vector<std::string> data_header_;
    void load_log_data(Eigen::MatrixXd data, std::string name);
    void load_log_data(double data, std::string name);
    void write_log_header();
    bool data_header_loaded_;
};

#endif