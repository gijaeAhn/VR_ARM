//
// Created by gj on 24. 3. 5.
//

#ifndef UTILITIES_TIMER_HPP
#define UTILITIES_TIMER_HPP


#include <chrono>
#include <thread>

namespace utilities{

struct Timer{

    int actualFrequency_;
    float realFrequency_;
    float dt_;

    std::chrono::steady_clock::time_point start_freq_;
    std::chrono::steady_clock::time_point end_freq_;
    std::chrono::steady_clock::time_point next_execution;

    std::chrono::milliseconds interval_;
    

    //Change Frequency by setting dt_
    Timer()
    : actualFrequency_(200),dt_(3),interval_(5){}
    Timer(int actual , float dt)
    : actualFrequency_(actual), dt_(dt) {}


    void start() {
        start_freq_ = std::chrono::steady_clock::now();
    }

    void stop() {
        end_freq_ = std::chrono::steady_clock::now();
    }
    void wait() {
        next_execution += interval_;
        std::this_thread::sleep_until(next_execution);
    }

    double getDuration() const {
        std::chrono::duration<double> elapsed_seconds = end_freq_ - start_freq_;
        return elapsed_seconds.count();
    }
};


}

#endif //UTILITIES_TIMER_HPP
