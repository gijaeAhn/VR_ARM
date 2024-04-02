//
// Created by gj on 24. 3. 5.
//

#ifndef VR_ARM_MOTOR_HPP
#define VR_ARM_MOTOR_HPP

#include <string>
#include <stdint.h>





namespace motor_interface {


    class Motor: {

        public :

        Motor(std::string const& type, uint8_t const id, std::string name, float tc);
        Motor() = delete;



        private :

        uint8_t id_;
        std::string type_;
        std::string name_;
        float tc_;
    };



    Motor::Motor(std::string const& type, uint8_t const id, std::string name, float tc)
    : id_(id), type_(type) {
    }

}

#endif //VR_ARM_MOTOR_DRIVER_HPP

   
