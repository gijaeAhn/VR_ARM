//
// Created by GJ on 5/21/24.
//
#include "utilities/include/address.hpp"
#include "utilities/include/timer.hpp"
#include "vr_interface/ControllerListener.hpp"

#include <stdint.h>
#include <string>
#include <cstdint>
#include <memory>
#include <zmq.hpp>





int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControllerListener>());
    rclcpp::shutdown();
    return 0;

}