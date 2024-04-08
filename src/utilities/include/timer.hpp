//
// Created by gj on 24. 3. 5.
//

#ifndef VR_ARM_TIMER_HPP
#define VR_ARM_TIMER_HPP


#include <chrono>

namespace utilities{

struct Timer{

    int actualFrequency_;
    double realFrequency_;

    std::chrono::steady_clock::time_point start_freq_;
    std::chrono::steady_clock::time_point end_freq_;
    std::chrono::steady_clock::time_point next_execution;

    std::chrono::milliseconds interval_;
    
    double dt_;

    //Change Frequency by setting dt_
    Timer()
    : actualFrequency_(200), dt_(0.003), interval_(5){}
    Timer(int actual , double dt)
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

#endif //VR_ARM_TIMER_HPP
