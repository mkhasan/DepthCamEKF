#ifndef _LOG_H 
#define _LOG_H 


#include <iostream>
#include <string>
#include <cstdio>
#include <cstdarg>

using namespace std;
namespace Log {

    constexpr const char* RESET  = "\033[0m";
    constexpr const char* RED    = "\033[31m";
    constexpr const char* GREEN  = "\033[32m";
    constexpr const char* YELLOW = "\033[33m";
    constexpr const char* CYAN   = "\033[36m";



    enum class Level {
        DEBUG,
        INFO,
        WARN,
        ERROR
    };

    inline  Level level = Level::INFO;

    inline void set_level(Level level_) {
        level = level_;

    }

    inline const char* color(Level level) {
        switch (level) {
            case Level::DEBUG: return CYAN;
            case Level::INFO:  return GREEN;
            case Level::WARN:  return YELLOW;
            case Level::ERROR: return RED;
            default:           return RESET;
        }
    }

    inline const char* name(Level level) {
        switch (level) {
            case Level::DEBUG: return "DEBUG";
            case Level::INFO:  return "INFO";
            case Level::WARN:  return "WARN";
            case Level::ERROR: return "ERROR";
            default:           return "LOG";
        }
    }

    inline void log(Level level_, const char* fmt, ...) {
        //cout << "log level here in: " << (int)Log::level << endl;
        if (level > level_) return;
        std::cout << color(level_);
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
        std::cout << RESET << std::endl;;
    }
    inline void log(Level level_, const std::string& msg) {
        if (level > level_) return;
        std::cout
            << color(level_)
            << "[" << name(level_) << "] "
            << msg
            << RESET
            << std::endl;
    }
}

#endif