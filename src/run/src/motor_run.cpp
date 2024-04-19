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
    rtp << "tcp://*:" << RMD_MOTOR_ADDR;
    dtp << "tcp://*:" << DYNAMIXEL_MOTOR_ADDR;

    rmd_torque_publisher.bind(rtp.str());
    dynamixel_torque_publisher.bind(dtp.str());
    zmq::sockopt::array_option<ZMQ_SUBSCRIBE,0> sockopt;
    motor_torque_subscriber.set(sockopt,"");

    std::cout << "Debug 1" << std::endl;


    while(true){
        std::cout << "Debug 2" << std::endl;

        auto start = std::chrono::high_resolution_clock::now();

        zmq::message_t robot_torque_sub_message(sizeof(float)*ROBOT_MOTOR_SIZE);


        zmq::recv_result_t recv_result =  motor_torque_subscriber.recv(robot_torque_sub_message, zmq::recv_flags::none);
        //Chekcing Recv here??
        if(!(recv_result.has_value() && recv_result.value() > 0 )){
            std::cout << "Failed to recieve net torque Array" << std::endl;
        }

        auto robot_torque_ptr = static_cast<float*>(robot_torque_sub_message.data());


        std::array<float,RMD_MOTOR_SIZE> rmd_torque;
        std::array<float,DYNAMIXEL_MOTOR_SIZE> dyna_torque;

        std::copy(rmd_torque.begin(),rmd_torque.end(),robot_torque_ptr);
        std::copy(dyna_torque.begin(),dyna_torque.end(),robot_torque_ptr + RMD_MOTOR_SIZE);

        zmq::message_t rmd_torque_pub_message(rmd_torque.data(),rmd_torque.size()*sizeof(float));
        zmq::message_t dynamixel_torque_pub_message(dyna_torque.data(),dyna_torque.size()*sizeof(float));

        zmq::send_result_t rmd_send_result = rmd_torque_publisher.send(rmd_torque_pub_message,zmq::send_flags::none);
        zmq::send_result_t dyna_send_result = dynamixel_torque_publisher.send(dynamixel_torque_pub_message,zmq::send_flags::none);

        if(!((rmd_send_result.has_value() && rmd_send_result.value() > 0) && (dyna_send_result.has_value() && dyna_send_result.value() > 0) )){
            std::cout << "Failed to send torque Array" << std::endl;
            
        }

        //Sending Check here?

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double frequency = 1.0 / elapsed.count();

        std::cout << "Actual frequency: " << frequency << " Hz" << std::endl;
    }

}