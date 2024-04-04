//
// Created by gj on 24. 3. 5.
//



#include "motor_interface.hpp"


namespace motor_interface{


    MotorInterface::MotorInterface(MotorConfig const& config): motorconfig_(config){

    }





    void MotorInterface::setTorque(std::array<float,ROBOT_MOTOR_SIZE> torque_list){

        float rmd_temp[RMD_MOTOR_SIZE];
        float dynamixel_temp[RMD_MOTOR_SIZE];

        //Suppose to get Torque from Calc 
        
        for(int i =0; i<RMD_MOTOR_SIZE;i++){
            rmd_temp[i] = torque_list[i];
        }
        for(int i = RMD_MOTOR_SIZE; i < DYNAMIXEL_MOTOR_SIZE; i++){
            dynamixel_temp[i] = torque_list[i];
        }
        


        RMD_TORQUE_SHM.SHM_WRITE(rmd_temp);
        DYNAMIXEL_TORQUE_SHM.SHM_WRITE(dynamixel_temp);
        RMD_DEBUG_SHM.SHM_READ(&rmd_debug_);
        DYNAMIXEL_DEBUG_SHM.SHM_READ(&dynamixel_debug_);


        displayDebug(rmd_debug_,dynamixel_debug_);
    }

    void MotorInterface::displayDebug(const uint8_t rmd_D, const uint8_t dyna_D){

        if(rmd_D | dyna_D == 0){
            //Add Color here
            printf("NP");
        }
        else if ((rmd_D && dyna_D) == 1){
            printf("Both have problem");
        }
        else if (rmd_D == 1 ){
            printf("RMD has problem");
        }
        else {
            printf("DYNAMIXEL has problem");
        }
    }
}
