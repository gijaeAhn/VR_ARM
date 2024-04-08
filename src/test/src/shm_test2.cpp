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
    RMD_TORQUE_TEST_SHM.SHM_CREATE();


    float rmd_torque_test_buffer[RMD_MOTOR_KEY] = {0};

    while(true){



        rmd_torque_test_buffer[0] += 1;

        RMD_TORQUE_TEST_SHM.SHM_WRITE(rmd_torque_test_buffer);

        
     
    }

}

void signalHandler(int signum) {
    printf("Interrupt siganl (%d) recived\n", signum);
    RMD_TORQUE_TEST_SHM.SHM_FREE();
    exit(EXIT_SUCCESS);
}

