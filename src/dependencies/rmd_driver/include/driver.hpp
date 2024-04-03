//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_DRIVER_HPP
#define RMD_DRIVER_DRIVER_HPP

#include <chrono>
#include <cstdint>
#include <string>

#include "protocol/node.hpp"
#include "protocol/request.hpp"
#include "protocol/response.hpp"

#include "feedback.hpp"
#include "motor_status.hpp"
#include "exceptions.hpp"

namespace rmd_driver{
    class Driver: protected Node {
    public:
        Driver(std::string const& ifname);
        Driver() = delete;
        Driver(Driver const&) = delete;
        Driver& operator = (Driver const&) = default;
        Driver(Driver&&) = default;
        Driver& operator = (Driver&&) = default;


        /**\fn addMotor
         * Adding Motor to MotroList
         * @param actuator_id
         * @return void
         */

        void addMotor(std::uint32_t actuator_id);


        /**\fn MotorRunning
         * Send MotorRunning Signal to specific motor
         * Depends on Motor Version, This function is optional
         * @param actuator_id
         * @return void
         */
        void MotorRunning(std::uint32_t actuator_id);

        [[nodiscard]]
        Feedback sendTorqueSetpoint(std::uint32_t actuator_id, float const current);


        void shutdownMotor(std::uint32_t actuator_id);

        
        void stopMotor(std::uint32_t actuator_id);



    };
}

#endif //RMD_DRIVER_DRIVER_HPP
