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



#define MOTOR_ID_OFFSET 1
#define MOTOR_INIT_TIME 1
#define DEBUQ_FREQ 500



//////// DYNAMIXEL CONFIGURATION

// Control table address

#define ADDR_TORQUE_ENABLE              64
#define ADDR_CURRENT_GOAL               102
#define ADDR_PRESENT_POSITION           132


// Data Byte Length
#define LEN_CURRENT_GOAL                2
#define LEN_PRESENT_POSITION            4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 11
#define DXL2_ID                         2                   // Dynamixel#2 ID: 12
#define DXL3_ID                         13                   // Dynamixel#3 ID: 13

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define TORQUE_CONSTANT                 2.12
#define CURRENT_UNIT                    2.69

#define ESC_ASCII_VALUE                 0x1b
#define DYNA_DEVICE_NAME                "/dev/ttyUSB0"
#define RMD_DEVICE_NAME                 "can0"


namespace driver{

    class integratedDriver{

    public :

        integratedDriver();
        ~integratedDriver();

        void rmdRunningFunc();
        void dynaRunningFunc();






    private:
        rmd_driver::Driver driver_;
        std::atomic<bool> running_;
        std::mutex rmdMutex_;
        std::mutex dynaMutex_;


        //RMD SIDE
        std::uint16_t freqCalc = 0;
        std::uint16_t freqSample = DEBUQ_FREQ;
        std::array<rmd_driver::Feedback,RMD_MOTOR_SIZE> bufFeed;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> currentShaft;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> previousShaft;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> shaftChange;
        std::array<SYSTEM_PRECISION_TYPE,RMD_MOTOR_SIZE> rmdTorqueBuffer;

        //DYNA SIDE
        std::array<SYSTEM_PRECISION_TYPE,DYNAMIXEL_MOTOR_SIZE> Buffer;
        std::array<SYSTEM_PRECISION_TYPE,DYNAMIXEL_MOTOR_SIZE> angBuffer;
        std::array<SYSTEM_PRECISION_TYPE,DYNAMIXEL_MOTOR_SIZE> currentBuffer;
        std::array<int16_t,DYNAMIXEL_MOTOR_SIZE> unitCurrentBuffer;
        std::array<uint32_t,DYNAMIXEL_MOTOR_SIZE> unitAngBuffer;
        std::array<uint32_t,DYNAMIXEL_MOTOR_SIZE> homePosition;
        sumBuffer.fill(0);




        //ENTIRE VARIABLES
        std::array<SYSTEM_PRECISION_TYPE,DOF> robotTorque_;
        std::array<SYSTEM_PRECISION_TYPE,DOF> robotAngle_;






    };






}




#endif //VR_ARM_INTEGRATED_DRIVER_HPP
