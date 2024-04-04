//
// Created by gj on 24. 4.2
//

#ifndef VR_ARM_MOTOR_CONFIG_HPP
#define VR_ARM_MOTOR_CONFIG_HPP

#include "dependencies/rmd_driver/include/driver.hpp"
#include "dependencies/dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"
#include "motor_interface/include/motor.hpp"
#include "utilities/include/shm.hpp"


#include <string>
#include <stdint.h>
#include <array>
#include <algorithm>

namespace motor_interface {


    class MotorConfig: {

        public :
        MotorConfig();
        
        

        /**\fn appendMotor
         * Append Motor to Motorlist
         * @param motor
         * return void
         */
        void setMotor(Motor& motor);

    

        private :

        std::array<std::string,ROBOT_MOTOR_SIZE> MotorType_list_;
        std::array<std::string,ROBOT_MOTOR_SIZE> MotorName_list_;
        std::array<uint8_t,ROBOT_MOTOR_SIZE> MotorID_list_;
        std::array<uint8_t,ROBOT_MOTOR_SIZE> MotorTC_list_;
    

    };



}

#endif //VR_ARM_MOTOR_CONFIG_HPP

   
