//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_REQUEST_HPP
#define RMD_DRIVER_REQUEST_HPP
#pragma once

#include "command_type.hpp"
#include "message.hpp"
#include "single_motor_message.hpp"
#include "exceptions.hpp"

namespace rmd_driver{



    class SetTorqueRequest: public SingleMotorRequest<CommandType::TORQUE_CLOSED_LOOP_CONTROL> {
    public:
        /**\fn SetTorqueRequest
         * \brief
         *    Class constructor
         *
         * \param[in] current
         *    The current set-point in Ampere [-20.00, 20.00]
        */
        SetTorqueRequest(float const current);
        SetTorqueRequest() = delete;
        SetTorqueRequest(SetTorqueRequest const&) = default;
        SetTorqueRequest& operator = (SetTorqueRequest const&) = default;
        SetTorqueRequest(SetTorqueRequest&&) = default;
        SetTorqueRequest& operator = (SetTorqueRequest&&) = default;
        using SingleMotorRequest::SingleMotorRequest;

        /**\fn getTorqueCurrent
         * \brief
         *    Get the torque current
         *
         * \return
         *    The torque current in Ampere [-20.00, 20.00]
        */
        [[nodiscard]]
        float getTorqueCurrent() const noexcept;
    };
    

    SetTorqueRequest::SetTorqueRequest(float const current)
    : SingleMotorRequest{} {
        //On work
        // auto const c {static_cast<std::int16_t>(current/0.01f)};
        auto const c {static_cast<std::int16_t>(current)};
        setAt(c, 4);
        return;
    }

    float SetTorqueRequest::getTorqueCurrent() const noexcept {
    // return static_cast<float>(getAs<std::int16_t>(4))*0.01f;
    return static_cast<float>(getAs<std::int16_t>(4));

    }




    using MotorRunningRequest = SingleMotorRequest<CommandType::MOTOR_RUNNING_COMMAND>;
    using ShutdownMotorRequest = SingleMotorRequest<CommandType::SHUTDOWN_MOTOR>;
    using StopMotorRequest = SingleMotorRequest<CommandType::STOP_MOTOR>;

}



#endif //RMD_DRIVER_REQUEST_HPP
