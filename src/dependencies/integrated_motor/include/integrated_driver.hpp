//
// Created by sj on 24. 7. 1.
//
#ifndef VR_ARM_INTEGRATED_DRIVER_HPP
#define VR_ARM_INTEGRATED_DRIVER_HPP

#include "dependencies/rmd_driver/include/rmd_driver.hpp"
#include "dependencies/dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"

#include "utilities/include/timer.hpp"
#include "utilities/include/address.hpp"
#include "utilities/include/param.hpp"
#include "utilities/include/debug.hpp"

#include <stdint>
#include <stdlib.h>
#include <memory>
#include <mutex>
#include <atomic>
#include <array>
#include <algorithm>
#include <chrono>
#include <zmq.hpp>
#include <thread>
#include <math.h>
#include <string>


//////// RMD CONFIGURATION
#define MOTOR_ID_OFFSET 1
#define MOTOR_INIT_TIME 1
#define RMD_DEBUQ_FREQ 500
#define RMD_CONTROL_FREQ 500
#define RMD_DEVICE_NAME "can0"
//////// RMD CONFIGURATION END

//////// DYNAMIXEL CONFIGURATION
#define ADDR_TORQUE_ENABLE              64
#define ADDR_CURRENT_GOAL               102
#define ADDR_PRESENT_POSITION           132

#define LEN_CURRENT_GOAL                2
#define LEN_PRESENT_POSITION            4

#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define DXL1_ID                         11                  // Dynamixel#1 ID: 11
#define DXL2_ID                         12                  // Dynamixel#2 ID: 12
#define DXL3_ID                         13                  // Dynamixel#3 ID: 13

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define TORQUE_CONSTANT                 2.12
#define CURRENT_UNIT                    2.69

#define ESC_ASCII_VALUE                 0x1b
#define DYNA_DEVICE_NAME                "/dev/ttyUSB0"
#define DYNA_CONTROL_FREQ               500

/////// DYNAMIXEL CONFIGURATION END

#define RUNNING_ON                      true
#define RUNNING_OFF                     false


namespace driver{

    class integratedDriver{

    public :

        integratedDriver();
        ~integratedDriver();

        void run();
        void terminate();

    private:
        rmd_driver::Driver driver_;
        std::atomic<bool> running_;
        std::mutex rmdMutex_;
        std::mutex dynaMutex_;
        std::mutex torqueMutex_;

        std::thread dynaThread_;
        std::thread rmdThread_;
        std::thread torqueThread_;

        void rmdRunningFunc();
        void dynaRunningFunc();
        void torqueSubFunc();

        //ROBOT SIDE
        std::array<SYSTEM_PRECISION_TYPE,ROBOT_MOTOR_SIZE> robotTorque_;
        std::array<SYSTEM_PRECISION_TYPE,ROBOT_ANGLE_ADDR> robotAngle_;


        //RMD SIDE
        std::uint16_t freqCalc = 0;
        std::uint16_t freqSample = DEBUQ_FREQ;
        std::array<rmd_driver::Feedback,RMD_MOTOR_SIZE>  rmdFeedBuf;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> rmdCurrentShaft;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> rmdPreviousShaft;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> rmdShaftChange;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> rmdTorque;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> rmdAngle;
        std::unique_ptr<zmq::socket_t> rmd_angle_publisher_;

        //DYNA SIDE
        std::array<SYSTEM_PRECISION_TYPE,DYNAMIXEL_MOTOR_SIZE> dynaBuffer;
        std::array<SYSTEM_PRECISION_TYPE,DYNAMIXEL_MOTOR_SIZE> dynaAngle;
        std::array<SYSTEM_PRECISION_TYPE,DYNAMIXEL_MOTOR_SIZE> dynaCurrent;
        std::array<int16_t,DYNAMIXEL_MOTOR_SIZE> dynaUnitCurrent;
        std::array<uint32_t,DYNAMIXEL_MOTOR_SIZE> dynaUnitAng;
        std::array<uint32_t,DYNAMIXEL_MOTOR_SIZE> dynaHomePosition;
        std::unique_ptr<zmq::socket_t> dyna_angle_publisher_;

        //ENTIRE VARIABLES
        std::array<SYSTEM_PRECISION_TYPE,DOF> robotTorque_;
        std::array<SYSTEM_PRECISION_TYPE,DOF> robotAngle_;
        std::unique_ptr<zmq::socket_t> robot_torque_subscriber_;

    };






}




#endif //VR_ARM_INTEGRATED_DRIVER_HPP
