//
// Created by gj on 24. 4. 3.
//




#include "dependencies/rmd_driver/include/rmd_driver.hpp"
#include "utilities/include/shm.hpp"

#include <vector>
#include <iostream>
#include <csignal>
#include <algorithm>
#include <cstdint>

utilities::memory::SHM<float> RMD_TORQUE_TEST_SHM(RMD_MOTOR_KEY,RMD_MOTOR_SIZE);



void signalHandler(int signal);


int main(int argc,char* argv[])
{
    std::signal(SIGINT,signalHandler);
    RMD_TORQUE_TEST_SHM.SHM_CREATE();
    float rmd_torque_test_buffer[RMD_MOTOR_SIZE] = {0};

    bool direction = true;


    while(true){


        rmd_torque_test_buffer[0] = 0;

        printf("Torque : %f \n", rmd_torque_test_buffer[0]);

        RMD_TORQUE_TEST_SHM.SHM_WRITE(rmd_torque_test_buffer);
        
        sleep(1);

        if(direction  == true){
            rmd_torque_test_buffer[0] += 1;
        }
        else if(direction == false){
            rmd_torque_test_buffer[0] -=1;

        }

        if(rmd_torque_test_buffer[0]>9){
            direction = false;
        }
        else if(rmd_torque_test_buffer[0]<1){
            direction = true;
        }
     
    }

}

void signalHandler(int signum) {
    printf("Interrupt siganl (%d) recived\n", signum);
    RMD_TORQUE_TEST_SHM.SHM_FREE();
    exit(EXIT_SUCCESS);
}

