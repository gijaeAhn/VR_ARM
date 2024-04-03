//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_SINGLE_MOTOR_MESSAGE_HPP
#define RMD_DRIVER_SINGLE_MOTOR_MESSAGE_HPP
#pragma once

#include <array>
#include <cstdint>

#include "command_type.hpp"
#include "message.hpp"
#include "can/can_exception.hpp"


namespace rmd_driver{


    template<CommandType C>
    class SingleMotorMessage : public Message{

    protected:
        SingleMotorMessage(std::array<std::uint8_t,8> const& data);
        SingleMotorMessage() ;
        SingleMotorMessage(SingleMotorMessage const&) = default;
        SingleMotorMessage& operator = (SingleMotorMessage const&) = default;
        SingleMotorMessage(SingleMotorMessage&&) = default;
        SingleMotorMessage& operator = (SingleMotorMessage&&) = default;
    };

    template <CommandType C>
    SingleMotorMessage<C>::SingleMotorMessage(std::array<std::uint8_t,8> const& data)
    : Message{data}{
        return;}

    template <CommandType C>
    SingleMotorMessage<C>::SingleMotorMessage() 
    : Message{} {
        data_[0] = static_cast<std::uint8_t>(C);
        return;}

    template <CommandType C>
    class SingleMotorRequest: public SingleMotorMessage<C> {
    public:
        constexpr SingleMotorRequest(std::array<std::uint8_t,8> const& data);
        constexpr SingleMotorRequest() = default;
        SingleMotorRequest(SingleMotorRequest const&) = default;
        SingleMotorRequest& operator = (SingleMotorRequest const&) = default;
        SingleMotorRequest(SingleMotorRequest&&) = default;
        SingleMotorRequest& operator = (SingleMotorRequest&&) = default;
    };

    template <CommandType C>
    constexpr SingleMotorRequest<C>::SingleMotorRequest(std::array<std::uint8_t,8> const& data)
            : SingleMotorMessage<C>{data} {
        return;
    }

    template <CommandType C>
    class SingleMotorResponse: public SingleMotorMessage<C> {
    public:
        constexpr SingleMotorResponse(std::array<std::uint8_t,8> const& data);
        constexpr SingleMotorResponse() = delete;
        SingleMotorResponse(SingleMotorResponse const&) = default;
        SingleMotorResponse& operator = (SingleMotorResponse const&) = default;
        SingleMotorResponse(SingleMotorResponse&&) = default;
        SingleMotorResponse& operator = (SingleMotorResponse&&) = default;
    };

    template <CommandType C>
    constexpr SingleMotorResponse<C>::SingleMotorResponse(std::array<std::uint8_t,8> const& data)
            : SingleMotorMessage<C>{data} {
        return;
    }
}

#endif //RMD_DRIVER_SINGLE_MOTOR_MESSAGE_HPP
