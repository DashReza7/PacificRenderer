#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    CRITICAL = 4
};

class Logger {
private:
    LogLevel currentLevel;
    std::ofstream logFile;
    bool logToConsole;
    bool logToFile;

    std::string levelToString(LogLevel level) {
        switch(level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::CRITICAL: return "CRITICAL";
            default: return "UNKNOWN";
        }
    }

    std::string getCurrentTime() {
        throw std::runtime_error("getCurrentTime not implemented");
    }

public:
    Logger(LogLevel level = LogLevel::INFO, 
           bool console = true, 
           const std::string& filename = "") 
        : currentLevel(level), logToConsole(console), logToFile(!filename.empty()) {
        
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

    template<typename... Args>
    void log(LogLevel level, const std::string& format, Args... args) {
        if (level < currentLevel) return;

        // FIXME:
        std::string message = std::format(format, args...);
        
        std::string logEntry = "[" + getCurrentTime() + "] [" + 
                              levelToString(level) + "] " + message;

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
#define LOG_CRITICAL(...) g_logger.log(LogLevel::CRITICAL, __VA_ARGS__)
