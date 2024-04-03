//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_MESSAGE_HPP
#define RMD_DRIVER_MESSAGE_HPP
#pragma once


#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <type_traits>



namespace rmd_driver{


    /**\class Message
      * \brief
      *    Base class for any message exchanged between the driver and the actuator
    */

    class Message {
    public:

        std::array<uint8_t,8> const& getData() const noexcept;
        void setData(std::array<uint8_t,8> const& data) noexcept;

    protected:
        Message(std::array<std::uint8_t,8> const& data = {}) noexcept;
        Message(Message const&) = default;
        Message& operator = (Message const&) = default;
        Message(Message&&) = default;
        Message& operator = (Message&&) = default;


        template <typename T, typename std::enable_if_t<std::is_integral_v<T>>* = nullptr>
        void setAt(T const val, std::size_t const i);


        template <typename T, typename std::enable_if_t<std::is_integral_v<T>>* = nullptr>
        T getAs(std::size_t const i) const;

        std::array<std::uint8_t,8> data_;
    };



    Message::Message(std::array<std::uint8_t,8> const& data) noexcept
    : data_{data} {
    return;
    }

    std::array<std::uint8_t,8> const& Message::getData() const noexcept {
    return data_;
    }

    void Message::setData(std::array<uint8_t, 8> const& data) noexcept {
        data_ = data;
    }


    template <typename T, typename std::enable_if_t<std::is_integral_v<T>>* = nullptr>
    void Message::setAt(T const val, std::size_t const i) {
        if (i + sizeof(T)/sizeof(std::uint8_t) > data_.size()) {
            throw std::out_of_range("Requested index out of range!");
        }
        std::memcpy(&data_[i], &val, sizeof(T));
        return;
    }

    template <typename T, typename std::enable_if_t<std::is_integral_v<T>>* = nullptr>
    T Message::getAs(std::size_t const i) const {
        if (i + sizeof(T)/sizeof(std::uint8_t) > data_.size()) {
            throw std::out_of_range("Requested index out of range!");
        }
        T val {};
        std::memcpy(&val, &data_[i], sizeof(T));
        return val;
    }


}

#endif //RMD_DRIVER_MESSAGE_HPP
