//
// Created by gj on 24. 4. 3.
//





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
    RMD_TORQUE_TEST_SHM.SHM_GETID();



    float rmd_torque_test_buffer = 0.0f;

    while(true){



        RMD_TORQUE_TEST_SHM.SHM_READ(&rmd_torque_test_buffer);
        printf("test : %f", rmd_torque_test_buffer);

        sleep(1);


        
     
    }

}

void signalHandler(int signum) {
    printf("Interrupt siganl (%d) recived\n", signum);
    RMD_TORQUE_TEST_SHM.SHM_FREE();
    exit(EXIT_SUCCESS);
}

