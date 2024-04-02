//
// Created by gj on 24. 3. 5.
//




#include "dependencies/rmd_driver/include/driver.hpp"
#include "dependencies/rmd_driver/include/feedback.hpp"
#include "dependencies/rmd_driver/include/meta/motorParam.hpp"


#include "utilities/include/shm.hpp"
#include "utilities/include/timer.hpp"



#include <stdint.h>
#include <string>
#include <vector>
#include <cstdint>
#include <array>

memory::SHM<float> RMD_TORQUE_SHM(RMD_MOTOR_KEY,RMD_MOTOR_SIZE);
memory::SHM<std::uint8_t> RMD_DEBUG_SHM(RMD_DEBUG_KEY);
memory::SHM<float> setSHM_ANGLE(ANGLE_KEY,ROBOT_MEM_SIZE);
memory::SHM<float> setSHM_VEL(VEL_KEY,ROBOT_MEM_SIZE);

void signalHandler(int signum);


int main(int argc, char *argv[])
{   
    std::string driver_name(argv[1])
    signal(SIGINT, signalHandler);
    robot::Timer timer;
    timer.next_execution = std::chrono::steady_clock::now();
    rmd_driver::Driver driver(driver_name);

    std::array<rmd_driver::Feedback,RMD_MOTOR_SIZE> buf_feed;
    buf_feed.resize(RMD_MOTOR_SIZE);


    std::array<uint8_t,RMD_MOTOR_SIZE> currentShaft(RMD_MOTOR_SIZE,0);
    std::array<uint8_t,RMD_MOTOR_SIZE> previousShaft(RMD_MOTOR_SIZE,0);

    std::array<float, RMD_MotorSize> pidBuffer{};
    std::array<float, RMD_MotorSize> gravBuffer{};
    std::array<float, RMD_MotorSize> sumBuffer{};
    std::array<float, RMD_MotorSize> angBuffer{};
    std::array<float, RMD_MotorSize> velBuffer{};
    std::array<float, RMD_MotorSize> shaftChange{};

    for (std::size_t i = 0; i < ROBOT_MEM_SIZE; ++i) {
        driver.addMotor(i + 1);
        driver.MotorRunning(i + 1);
        auto buf = myactuator_rmd::Feedback{driver.sendTorqueSetpoint(i + 1, 0)};
        previousShaft[i] = buf.shaft_angle; // Make sure previousShaft has at least ROBOT_MEM_SIZE elements
        velBuffer[i] = 0.0f;
        shaftChange[i] = 0.0f;
    }




}






