//
// Created by gj on 24. 3. 5.
//

#ifndef VR_ARM_MOTOR_DRIVER_HPP
#define VR_ARM_MOTOR_DRIVER_HPP

#include "dependencies/rmd_driver/include/driver.hpp"
#include "dependencies/dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"
#include "motor_config.hpp"
#include "utilities/include/shm.hpp"
#include "utilities/include/timer.hpp"

#include <string>
#include <vector>
#include <stdint.h>
#include <array>


namespace motor_interface {


    class MotorInterface: {

        public :

        MotorInterface(MotorConfig const& config) ;
        MotorInterface() = delete;
        MotorInterface(MotorInterface const&) = delete;
        MotorInterface& operator = (MotorInterface const&) = default;
        MotorInterface(MotorInterface&&) = default;


        void setTorque(std::array<float,ROBOT_MOTOR_SIZE> torque_list);
        void displayDebug();
        


        private :

        MotorConfig motorconfig_;
        utilities::memory::SHM<float> RMD_TORQUE_SHM;
        utilities::memory::SHM<float> DYNAMIXEL_TORQUE_SHM;
        utilities::memory::SHM<std::uint8_t> RMD_DEBUG_SHM;
        utilities::memory::SHM<std::uint8_t> DYNAMIXEL_DEBUG_SHM;

        std::array<float,ROBOT_MOTOR_SIZE> torque_list_;
        std::array<float,RMD_MOTOR_SIZE  >  rmd_set;
        std::array<float,DYNAMIXEL_MOTOR_SIZE>  dyna_set;
        utilities::Timer timer;

        uint8_t rmd_debug_[1];
        uint8_t dynamxiel_debug_[1];
    };



}

#endif //VR_ARM_MOTOR_DRIVER_HPP

   
