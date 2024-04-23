//
// Created by gj on 24. 3. 5.
//
#include "driver.hpp"
#include "feedback.hpp"
#include "meta/motorParam.hpp"

#include "utilities/include/timer.hpp"
#include "utilities/include/shm.hpp"

#include <cmath>
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
#include <sstream>


#define MOTOR_ID_OFFSET 1
#define MOTOR_INIT_TIME 1
#define DEBUQ_FREQ 500






void signalHandler(int signum)
{
    // std::cout << "Interrupt signal (" << signum << " ) recieved.\n" << std::endl; 
    printf("Interrupt signal (%d) recieve.\n", signum);
    std::exit(EXIT_SUCCESS);
}


int main(int argc, char* argv[])
{   
    //Params
    // 1. CAN Device name
    std::string device_name;
    if (argc > 1) { // Check that an argument was indeed passed
        device_name = argv[1];
        std::cout << "Device name: " << device_name << std::endl;
    } else {
        std::cout << "No device name provided." << std::endl;
        return EXIT_FAILURE;
    }



    std::signal(SIGINT, signalHandler);

    //ZMQ Node SET
    zmq::context_t context(1);
    zmq::socket_t rmd_torque_subscriber(context,ZMQ_SUB);
    zmq::socket_t rmd_angle_publisher(context,ZMQ_PUB);

    //ZMQ Option SET
    std::stringstream rts;
    std::stringstream rap;
    std::string host = "localhost";
    rts << "tcp://" << host << ":" << RMD_MOTOR_ADDR;
    rap << "tcp://*:" << RMD_ANGLE_ADDR;
    rmd_torque_subscriber.connect(rts.str());
    rmd_angle_publisher.bind(rap.str());
    zmq::sockopt::array_option<ZMQ_SUBSCRIBE,0> sockopt;
    rmd_torque_subscriber.set(sockopt,"");

    //ZMQ Message SET
    zmq::message_t torque_sub_message(sizeof(float)*RMD_MOTOR_SIZE);
    zmq::message_t angle_pub_message(sizeof(float)*RMD_MOTOR_SIZE);


    //Timer and Buffer
    utilities::Timer timer;
    timer.next_execution = std::chrono::steady_clock::now();
    rmd_driver::Driver driver(device_name);
    std::uint16_t freqCalc = 0;
    std::uint16_t freqSample = DEBUQ_FREQ;

    std::array<rmd_driver::Feedback,RMD_MOTOR_SIZE> bufFeed;
    std::array<float,RMD_MOTOR_SIZE> currentShaft  ;
    std::array<float,RMD_MOTOR_SIZE> previousShaft ;
    std::array<float,RMD_MOTOR_SIZE> shaftChange  ;
    std::array<float,RMD_MOTOR_SIZE> pidBuffer;
    std::array<float,RMD_MOTOR_SIZE> gravBuffe;
    std::array<float,RMD_MOTOR_SIZE> sumBuffer;
    std::array<float,RMD_MOTOR_SIZE> angBuffer;
    std::int16_t sendTorque = 0;
    currentShaft.fill(0);
    previousShaft.fill(0);
    shaftChange.fill(0);
    pidBuffer.fill(0);
    gravBuffe.fill(0);
    sumBuffer.fill(0);
    angBuffer.fill(0);
    
    // INIT MOTORS
    // SLEEP MOTOR * MOTOR INIT TIME
    for (uint8_t index = 0; index < RMD_MOTOR_SIZE; index++) {
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

    //Handle of While loop should be modified.
    while(true){
        freqCalc +=1;
        //::Read Torque here::
        zmq::recv_result_t result = rmd_torque_subscriber.recv(torque_sub_message,zmq::recv_flags::none);
        if (!result) {
        std::cerr << "Failed to receive message: " << std::endl;
        }
        std::copy(sumBuffer.begin(),sumBuffer.end(),static_cast<float*>(torque_sub_message.data()));

            for(uint8_t index=0; index < RMD_MOTOR_SIZE; index++)
            {
                sumBuffer[index] = std::clamp(sumBuffer[index], -10.0f, 10.0f);
                // Motor Constant
                // 9.8T / 3.0 A = 3.267
                // int16t range -2000 ~ 2000
                // to -32A to 32A 
                // 
                // Torque to unit
                // Torque / 3.267 = Current
                // Sending Value = Torque * 19.131;

                //Rounding Value;
                sendTorque = static_cast<int16_t>(std::round(sumBuffer[index] * 19.131f));
                bufFeed[index] = driver.sendTorqueSetpoint(index+MOTOR_ID_OFFSET,sendTorque);
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

        //::Send Angle here::
        if (angle_pub_message.size() >= sizeof(float) * RMD_MOTOR_SIZE) {
        //zmq::message_t data type == void*
        //Cause std::copy type problem 
        float* temp_message_ptr = static_cast<float*>(angle_pub_message.data());
        std::copy(temp_message_ptr,temp_message_ptr + RMD_MOTOR_SIZE,angBuffer.begin());
        rmd_angle_publisher.send(angle_pub_message,zmq::send_flags::none);
        } else {
            // Handle error: Not enough data
            std::cerr << "Error: message does not contain enough data." << std::endl;
        }



        if (freqCalc >= freqSample)
        {   
            std::cout << "Recieved Torque : " << sumBuffer[0]  << std::endl;
            auto cycleEndtime = std::chrono::steady_clock::now();
            std::chrono::duration<float> elapsed = cycleEndtime - cycleStarttime;
            float frequency = static_cast<float>(freqSample) / elapsed.count(); 
            freqCalc = 0;
            std::cout<< "Actual Frequency : " << frequency << std::endl;
            cycleStarttime = std::chrono::steady_clock::now();
        }
    }

}









