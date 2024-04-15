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




void signalHandler(int signal);


int main(int argc,char* argv[])
{
    std::signal(SIGINT,signalHandler);
    float rmd_torque_test_buffer[RMD_MOTOR_SIZE] = {0};

    bool direction = true;

    rmd_torque_test_buffer[0] = 0;

    while(true){



        printf("Torque : %f \n", rmd_torque_test_buffer[0]);

        
        sleep(1);

        for(int i = 0 ;i <RMD_MOTOR_SIZE ; i++)
        {   
            if(direction == true){
            rmd_torque_test_buffer[i] += 3;}


            if(direction == false )
            rmd_torque_test_buffer[i] += -3;

            if(abs(rmd_torque_test_buffer[i]) >10)
            direction = !direction;
        }

        
     
    }

}

void signalHandler(int signum) {
    printf("Interrupt siganl (%d) recived\n", signum);
    exit(EXIT_SUCCESS);
}

