#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <thread>

namespace exotica
{
inline void sleep(double t)
{
    std::this_thread::sleep_for(std::chrono::duration<double>(t));
}

class Timer
{
public:
    Timer() : startTime_(std::chrono::high_resolution_clock::now())
    {
    }

    inline void reset()
    {
        startTime_ = std::chrono::high_resolution_clock::now();
    }

    inline double getDuration()
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - startTime_).count();
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
};
}

#endif  // TIMER_H
