#pragma once
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>


enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
};

class Logger {
private:
    LogLevel currentLevel;
    std::ofstream logFile;
    bool logToConsole;
    bool logToFile;
    std::mutex logMutex;

    std::string levelToString(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG:
                return "DEBUG";
            case LogLevel::INFO:
                return "INFO";
            case LogLevel::WARNING:
                return "WARNING";
            case LogLevel::ERROR:
                return "ERROR";
            default:
                return "UNKNOWN";
        }
    }

public:
    Logger(LogLevel level = LogLevel::INFO, bool console = true, const std::string& filename = "") : currentLevel(level), logToConsole(console), logToFile(!filename.empty()) {
        if (logToFile)
            logFile.open(filename, std::ios::app);
    }
    ~Logger() {
        if (logFile.is_open())
            logFile.close();
    }

    void setLevel(LogLevel level) {
        currentLevel = level;
    }

    static std::string getCurrentTime() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        #ifdef _WIN32
            localtime_s(&tm_buf, &in_time_t);
        #else
            localtime_r(&in_time_t, &tm_buf);
        #endif
        std::stringstream ss;
        ss << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S");
        return ss.str();
    }

    template <typename... Args>
    void log(LogLevel level, const std::string& format, Args... args) {
        if (level < currentLevel) return;

        std::lock_guard<std::mutex> lock(logMutex);

        std::string message = std::vformat(format, std::make_format_args(args...));

        std::string logEntry = "[" + getCurrentTime() + "] [" + levelToString(level) + "] " + message;

        if (logToConsole) {
            if (level >= LogLevel::ERROR)
                std::cerr << logEntry << std::endl;
            else
                std::cout << logEntry << std::endl;
        }

        if (logToFile && logFile.is_open()) {
            logFile << logEntry << std::endl;
            logFile.flush();
        }
    }
};

// Global logger instance
extern Logger g_logger;

// Convenience macros
#define LOG_DEBUG(...) g_logger.log(LogLevel::DEBUG, __VA_ARGS__)
#define LOG_INFO(...) g_logger.log(LogLevel::INFO, __VA_ARGS__)
#define LOG_WARNING(...) g_logger.log(LogLevel::WARNING, __VA_ARGS__)
#define LOG_ERROR(...) g_logger.log(LogLevel::ERROR, __VA_ARGS__)

inline Logger createLogger() {
    std::string curr_time = Logger::getCurrentTime();
    std::string log_dir = "logs";
    std::string log_file = log_dir + "/log_" + curr_time + ".txt";
    
    // Create logs directory if it doesn't exist
    if (!std::filesystem::exists(log_dir)) {
        std::filesystem::create_directories(log_dir);
    }
    if (std::filesystem::exists(log_file)) {
        throw std::runtime_error("Log file already exists: " + log_file);
    }
    
    return Logger{LogLevel::INFO, true, log_file};
}
