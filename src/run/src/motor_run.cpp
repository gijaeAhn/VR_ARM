#include "utilities/include/timer.hpp"
#include "utilities/include/address.hpp"
#include "utilities/include/param.hpp"


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



int main(){

    std::cout << "Start Motor Run" << std::endl;


    //ZMQ Node SET
    zmq::context_t context(1);
    zmq::socket_t motor_torque_subscriber(context,ZMQ_SUB);
    zmq::socket_t rmd_torque_publisher(context,ZMQ_PUB);
    zmq::socket_t dynamixel_torque_publisher(context,ZMQ_PUB);

    //ZMQ Option SET
    std::stringstream mts;
    std::stringstream rtp;
    std::stringstream dtp;
    std::string host = "localhost";

    mts << "tcp://" << host << ":" <<  MOTOR_ADDR;
    rtp << "tcp://*:" << RMD_MOTOR_ADDR;
    dtp << "tcp://*:" << DYNAMIXEL_MOTOR_ADDR;


    motor_torque_subscriber.connect(mts.str());
    rmd_torque_publisher.bind(rtp.str());
    dynamixel_torque_publisher.bind(dtp.str());
    
    zmq::sockopt::array_option<ZMQ_SUBSCRIBE,1> sockopt;
    motor_torque_subscriber.set(sockopt,"");

    std::array<float,ROBOT_MOTOR_SIZE> debugBuffer;
    std::array<float,RMD_MOTOR_SIZE> rmd_torque;
    std::array<float,DYNAMIXEL_MOTOR_SIZE> dyna_torque;
    debugBuffer.fill(0);
    rmd_torque.fill(0);
    dyna_torque.fill(0);




    while(true){

        auto start = std::chrono::high_resolution_clock::now();

        zmq::message_t robot_torque_sub_message(sizeof(float)*ROBOT_MOTOR_SIZE);
        auto temp_ptr = reinterpret_cast<float*>(robot_torque_sub_message.data());


        zmq::recv_result_t recv_result =  motor_torque_subscriber.recv(robot_torque_sub_message, zmq::recv_flags::none);
        //Chekcing Recv here??
        if(!(recv_result.has_value() && recv_result.value() > 0 )){
            std::cout << "Failed to recieve net torque Array" << std::endl;
        }
        //Debuging ZMQ
        else {
            std::copy(temp_ptr,temp_ptr+ROBOT_MOTOR_SIZE,debugBuffer.begin());
            for (const auto& torque : debugBuffer) {
                std::cout << torque << " ";
            }
            std::cout << std::endl;

            std::cout << "debugBuffer Size : " << debugBuffer.size() << " " << "message size : " << robot_torque_sub_message.size() 
            << std::endl;
        }

        auto robot_torque_ptr = static_cast<float*>(robot_torque_sub_message.data());




        //Send RMD, Dyna torques
        std::copy(robot_torque_ptr,robot_torque_ptr + RMD_MOTOR_SIZE, rmd_torque.data());
        std::copy(robot_torque_ptr+RMD_MOTOR_SIZE,robot_torque_ptr+ROBOT_MOTOR_SIZE,dyna_torque.data());

        for(uint8_t index = 0; index < RMD_MOTOR_SIZE; index++){
            printf("  RMD Motor Torque %d : %f  ",index+1,rmd_torque[index]);
            if(index + 1 == DYNAMIXEL_MOTOR_SIZE){
                printf("\n");
            }
        }

        for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
            printf("  DYNA Motor Torque %d : %f  ",index+1,dyna_torque[index]);
            if(index + 1 == DYNAMIXEL_MOTOR_SIZE){
                printf("\n");
            }
        }

        zmq::message_t rmd_torque_pub_message(rmd_torque.data(),rmd_torque.size()*sizeof(float));
        zmq::message_t dynamixel_torque_pub_message(dyna_torque.data(),dyna_torque.size()*sizeof(float));

        zmq::send_result_t rmd_send_result = rmd_torque_publisher.send(rmd_torque_pub_message,zmq::send_flags::none);
        zmq::send_result_t dyna_send_result = dynamixel_torque_publisher.send(dynamixel_torque_pub_message,zmq::send_flags::none);

        if(!((rmd_send_result.has_value() && rmd_send_result.value() > 0) && (dyna_send_result.has_value() && dyna_send_result.value() > 0) )){
            std::cout << "Failed to send torque Array" << std::endl;
            
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double frequency = 1.0 / elapsed.count();

        std::cout << "Actual frequency: " << frequency << " Hz" << std::endl;
    }

}