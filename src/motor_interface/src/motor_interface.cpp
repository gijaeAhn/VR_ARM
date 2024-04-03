//
// Created by gj on 24. 3. 5.
//



#include "motor_interface.hpp"


namespace motor_interface{


    MotorInterface::MotorInterface(MotorConfig const& config): motorconfig_(config){

    }





    void MotorInterface::setTorque(std::array<float,ROBOT_MOTOR_SIZE> torque_list){


        //Suppose to get Torque from Calc 
        
        for(int i =0; i<RMD_MOTOR_SIZE;i++){
            rmd_set[i] = torque_list_[i];
        }
        for(int i = RMD_MOTOR_SIZE; i < DYNAMIXEL_MOTOR_SIZE; i++){
            dyna_set[i] = torque_list_[i];
        }


        RMD_TORQUE_SHM.SHM_WRITE(rmd_temp);
        DYNAMIXEL_TORQUE_SHM.SHM_WRITE(temp);


        RMD_DEBUG_SHM.SHM_READ(rmd_debug_);
        DYNAMIXEL_DEBUG_SHM.SHM_READ(dynamixel_debug_);


        displayDebug();
    }

    void MotorInterface::displayDebug(){

        

    }
}
