//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_COMMAND_TYPE_HPP
#define RMD_DRIVER_COMMAND_TYPE_HPP

#pragma once

#include <cstdint>

namespace rmd_driver{

    enum class CommandType: std::uint8_t {
        READ_PID_PARAMETERS = 0x30,
        WRITE_PID_PARAMETERS_TO_RAM = 0x31,
        WRITE_PID_PARAMETERS_TO_ROM = 0x32,
        READ_ACCELERATION = 0x42,
        READ_MULTI_TURN_ENCODER_POSITION = 0x60,
        READ_MULTI_TURN_ENCODER_ORIGINAL_POSITION = 0x61,
        READ_MULTI_TURN_ENCODER_ZERO_OFFSET = 0x62,
        READ_SINGLE_TURN_ENCODER = 0x90,
        READ_MULTI_TURN_ANGLE = 0x92,
        READ_SINGLE_TURN_ANGLE = 0x94,
        READ_MOTOR_STATUS_1_AND_ERROR_FLAG = 0x9A,
        READ_MOTOR_STATUS_2 = 0x9C,
        READ_MOTOR_STATUS_3 = 0x9D,
        SHUTDOWN_MOTOR = 0x80,
        STOP_MOTOR = 0x81,
        TORQUE_CLOSED_LOOP_CONTROL = 0xA1,
        SPEED_CLOSED_LOOP_CONTROL = 0xA2,
        ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL = 0xA4,
        READ_SYSTEM_OPERATING_MODE = 0x70,
        READ_MOTOR_POWER = 0x71,
        RESET_SYSTEM = 0x76,
        RELEASE_BRAKE = 0x77,
        LOCK_BRAKE = 0x78,
        READ_SYSTEM_RUNTIME = 0xB1,
        READ_SYSTEM_SOFTWARE_VERSION_DATE = 0xB2,
        READ_MOTOR_MODEL = 0xB5,
        MOTOR_RUNNING_COMMAND = 0x88

    };

    // Symmetric comparison operators
    constexpr bool operator == (CommandType const& c, std::uint8_t const i) noexcept {
    return i == static_cast<std::uint8_t>(c);
    }
    constexpr bool operator == (std::uint8_t const i, CommandType const& c) noexcept {
    return operator == (c, i);
    }

    constexpr bool operator != (CommandType const& c, std::uint8_t const i) noexcept {
    return !(operator == (c, i));
    }
    constexpr bool operator != (std::uint8_t const i, CommandType const& c) noexcept {
    return operator != (c, i);
    }
}

#endif //RMD_DRIVER_COMMAND_TYPE_HPP
