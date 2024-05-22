//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_RESPONSE_HPP
#define RMD_DRIVER_RESPONSE_HPP
#pragma once

#include "command_type.hpp"
#include "message.hpp"
#include "single_motor_message.hpp"
#include "feedback.hpp"

namespace rmd_driver {



    template<CommandType C>
    class FeedbackResponse: public SingleMotorResponse<C> {

    public:
        FeedbackResponse() = delete;
        FeedbackResponse(FeedbackResponse const&) = default;
        FeedbackResponse& operator = (FeedbackResponse const&) = default;
        FeedbackResponse(FeedbackResponse&&) = default;
        FeedbackResponse& operator = (FeedbackResponse&&) = default;
        using SingleMotorResponse<C>::SingleMotorResponse;

        [[nodiscard]]
        Feedback getStatus() const noexcept;
    };

    template <CommandType C>
    Feedback FeedbackResponse<C>::getStatus() const noexcept {
    auto const temperature {static_cast<int>(this->template getAs<std::int8_t>(1))};
    auto const current {static_cast<float>(this->template getAs<std::int16_t>(2))*0.01f};
    auto const shaft_speed {static_cast<float>(this->template getAs<std::int16_t>(4))};
    auto const shaft_angle {static_cast<float>(this->template getAs<std::uint16_t>(6))};
    return Feedback{temperature, current, shaft_speed, shaft_angle};
    }

    using SetTorqueResponse = FeedbackResponse<CommandType::TORQUE_CLOSED_LOOP_CONTROL>;
    using ShutdownMotorResponse = SingleMotorResponse<CommandType::SHUTDOWN_MOTOR>;
    using StopMotorResponse = SingleMotorResponse<CommandType::STOP_MOTOR>;
    using MotorRunningResponse = SingleMotorResponse<CommandType::MOTOR_RUNNING_COMMAND>;





}
#endif //RMD_DRIVER_RESPONSE_HPP
