//
// Created by gj on 24. 3. 5.
//
#include "driver.hpp"
#include "feedback.hpp"
#include "meta/motorParam.hpp"

#include "utilities/include/timer.hpp"
#include "utilities/include/shm.hpp"


#include <stdint.h>
#include <string>
#include <vector>
#include <cstdint>
#include <array>
#include <signal.h>
#include <utility>
#include <algorithm>
#include <csignal>
#include <zmq.hpp>
#include <array>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <zmq.hpp>
#include <cstring>


#define MOTOR_ID_OFFSET 1
#define MOTOR_INIT_TIME 1






void signalHandler(int signum)
{
    // std::cout << "Interrupt signal (" << signum << " ) recieved.\n" << std::endl; 
    printf("Interrupt signal (%d) recieve.\n", signum);
    std::exit(EXIT_SUCCESS);
}


int main()
{       
    std::signal(SIGINT, signalHandler);

    //ZMQ Node SET
    zmq::context_t context(1);
    zmq::socket_t rmd_torque_subscriber(context,ZMQ_SUB);
    zmq::socket_t rmd_angle_publisher(context,ZMQ_PUB);


    rmd_torque_subscriber.connect("tcp://localhost:5555");
    rmd_angle_publisher.bind("tcp://*:5556");
    rmd_torque_subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);


    zmq::message_t angle_pub_mesage;
    zmq::message_t torque_sub_message;

    utilities::Timer timer;
    timer.next_execution = std::chrono::steady_clock::now();
    rmd_driver::Driver driver("can0");

    std::array<rmd_driver::Feedback,RMD_MOTOR_SIZE> bufFeed;
    float currentShaft[RMD_MOTOR_SIZE] ={0};
    float previousShaft[RMD_MOTOR_SIZE]={0};
    float shaftChange[RMD_MOTOR_SIZE]  ={0};

    // BUFFER FOR SHM
    float pidBuffer[RMD_MOTOR_SIZE]= {0};
    float gravBuffe[RMD_MOTOR_SIZE]= {0};
    float sumBuffer[RMD_MOTOR_SIZE]= {0};
    float angBuffer[RMD_MOTOR_SIZE]= {0};


    
    // INIT MOTORS
    // SLEEP MOTOR * MOTOR INIT TIME
    for (int index = 0; index < RMD_MOTOR_SIZE; index++) {
        driver.addMotor(index + MOTOR_ID_OFFSET);
        driver.MotorRunning(index + MOTOR_ID_OFFSET);
        printf("Motor %d running", index+MOTOR_ID_OFFSET);
        auto buf = rmd_driver::Feedback{driver.sendTorqueSetpoint(index + MOTOR_ID_OFFSET, 0)};
        previousShaft[index] = buf.getShaft(); // Make sure previousShaft has at least ROBOT_MEM_SIZE elements
        sleep(MOTOR_INIT_TIME);
        printf("Debug MOTOR INIT  %d", index);
    }
    



    

    //READ HOME ANGLE CONFIGURATION

    //FOR REALTIME FREQ CALC
    auto cycleStarttime = std::chrono::steady_clock::now(); 


    
    //System should be seized when signal come in
    while(true){

            for(int index=0; index < RMD_MOTOR_SIZE; index++)
            {
                sumBuffer[index] = std::clamp(sumBuffer[index], -10.0f, 10.0f);
                bufFeed[index] = driver.sendTorqueSetpoint(index+MOTOR_ID_OFFSET,sumBuffer[index]);
                previousShaft[index] = currentShaft[index];
                currentShaft[index] = bufFeed[index].getShaft();

                //Estimating current Angle

                auto delta = currentShaft[index] - previousShaft[index];
                if ( delta > 50000) {
                    shaftChange[index] = -((rmd_driver::maxShaftAngle - currentShaft[index]) + previousShaft[index]);
                }
                else if (delta < -50000) {
                    shaftChange[index] = currentShaft[index] + (rmd_driver::maxShaftAngle - previousShaft[index]);
                }
                else{
                    shaftChange[index] = delta;
                }
                
                angBuffer[index] += shaftChange[index]/(rmd_driver::maxShaftAngle)*(rmd_driver::oneShaftCycle);

            }

        RMD_ANGLE_SHM.SHM_WRITE(angBuffer);

        if (freqCalc >= freqSample)
        {   
            std::cout << "Recieved Torque : " << sumBuffer[0]  << std::endl;
            auto cycleEndtime = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = cycleEndtime - cycleStarttime;

            // Calculate frequency
            double frequency = static_cast<double>(freqSample) / elapsed.count(); 
            

            // Reset the counter and the start time for the next measurement
            freqCalc = 0;
            cycleStarttime = std::chrono::steady_clock::now();
        }
    }

}









