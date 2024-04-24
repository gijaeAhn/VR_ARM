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
    

    std::cout << "Motor Test Start" << std::endl;

    zmq::context_t context(1);
    zmq::socket_t torque_publisher(context,ZMQ_PUB);

    std::stringstream tp;
    std::string host = "localhost";

    tp << "tcp://*:" << MOTOR_ADDR;

    torque_publisher.bind(tp.str());

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


    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep for 0.1 seconds
}







}