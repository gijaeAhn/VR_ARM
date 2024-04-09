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

    int freq =0;
    while(true){
        freq+=1;
        if(freq > 1000000){
            printf("Working\n");
            freq = 0;
        }

    } 

}

void signalHandler(int signum) {
    printf("Interrupt siganl (%d) recived\n", signum);
    RMD_TORQUE_TEST_SHM.SHM_FREE();
    exit(EXIT_SUCCESS);
}

