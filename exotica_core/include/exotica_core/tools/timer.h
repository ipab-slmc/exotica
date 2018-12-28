#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <thread>

namespace exotica
{
inline void Sleep(double t)
{
    std::this_thread::sleep_for(std::chrono::duration<double>(t));
}

class Timer
{
public:
    Timer() : start_time_(std::chrono::high_resolution_clock::now())
    {
    }

    inline void Reset()
    {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    inline double GetDuration()
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_).count();
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};
}

#endif  // TIMER_H
