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
    

    std::cout << "Motor Test Start" << std::endl;

    zmq::context_t context(1);
    zmq::socket_t torque_publisher(context,ZMQ_PUB);
    zmq::socket_t rmd_angle_subscriber(context,ZMQ_SUB);
    zmq::socket_t dyna_angle_subscriber(context,ZMQ_SUB);

    std::stringstream tp;
    std::stringstream ras;
    std::stringstream das;
    std::string host = "localhost";
    zmq::sockopt::array_option<ZMQ_SUBSCRIBE,1> socketopt;

    tp << "tcp://*:" << MOTOR_ADDR;
    ras << "tcp://" << "localhost:"<< RMD_ANGLE_ADDR;
    das << "tcp://" << "localhost:" << DYNAMIXEL_ANGLE_ADDR;

    torque_publisher.bind(tp.str());
    rmd_angle_subscriber.connect(ras.str());
    dyna_angle_subscriber.connect(das.str());
    rmd_angle_subscriber.set(socketopt,"");
    dyna_angle_subscriber.set(socketopt,"");
    

    std::cout << "Debug 1" << std::endl;

    std::array<float,ROBOT_MOTOR_SIZE> temp_torque_array;
    std::cout << "Debug 2" << std::endl;

    temp_torque_array.fill(0);  
    bool increasing = true;  // Direction of torque change

    std::cout << "Debug 3" << std::endl;

    while (true) {
    if (increasing) {
        if (temp_torque_array[0] < 2.0) {  
            std::transform(temp_torque_array.begin(), temp_torque_array.end(), temp_torque_array.begin(),
                [](float torque) { return torque + 0.1; });
        } else {
            increasing = false;  
        }
    } else {
        if (temp_torque_array[0] > -2.0) {  
            std::transform(temp_torque_array.begin(), temp_torque_array.end(), temp_torque_array.begin(),
                [](float torque) { return torque - 0.1; });
        } else {
            increasing = true;  
        }
    }

    for (const auto& torque : temp_torque_array) {
        std::cout << torque << " ";
    }
    std::cout << std::endl;

    zmq::message_t motor_test_mesg(temp_torque_array.data(),sizeof(float)*ROBOT_MOTOR_SIZE) ;
    torque_publisher.send(motor_test_mesg,zmq::send_flags::none);

    zmq::message_t rmd_angle_mesg(sizeof(float)*RMD_MOTOR_SIZE);
    zmq::message_t dyna_angle_mesg(sizeof(float)*DYNAMIXEL_MOTOR_SIZE);

    size_t r_size = rmd_angle_mesg.size() / sizeof(float); // Calculate how many floats are in the message
    size_t d_size = dyna_angle_mesg.size() / sizeof(float); // Calculate how many floats are in the message

    auto r_data = static_cast<float*>(rmd_angle_mesg.data());
    auto d_data = static_cast<float*>(dyna_angle_mesg.data());


    for (size_t i = 0; i < r_size; ++i) {
        std::cout << "RMD Motor Angle : ";
        std::cout<< r_data[i] ;
    }
    for (size_t i = 0; i < d_size; ++i) {
        std::cout << "Dyna Motor Angle :  " ;
        std::cout <<  d_data[i] << " degrees";
    }
    std::cout << std::endl;


    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Sleep for 0.01 seconds
}







}