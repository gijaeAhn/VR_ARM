//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_FRAME_HPP
#define RMD_DRIVER_FRAME_HPP
#pragma once


#include <array>
#include <cstdint>

namespace rmd_driver{
    namespace can{

        class Frame{
            public:

            Frame(std::uint32_t const can_id, std::array<std::uint8_t,8> const& data,bool error) noexcept;
            Frame() = delete;
            Frame(Frame const&) = default;
            Frame& operator = (Frame const&) = default;
            Frame(Frame&&) = default;
            Frame& operator = (Frame&&) = default;

            [[nodiscard]]
            constexpr std::uint32_t getId() const noexcept;

            [[nodiscard]]
            constexpr std::array<std::uint8_t,8> getData() const noexcept;

            void setError() noexcept;


            protected:

            std::uint32_t can_id_;
            std::array<uint8_t,8> data_;
            bool error_;
        };

        inline Frame::Frame(std::uint32_t const can_id, std::array<uint8_t,8> const& data,bool error) noexcept
        : can_id_{can_id}, data_{data}, error_{error}{
            return;
        }

        constexpr std::uint32_t Frame::getId() const noexcept{
            return can_id_;
        }

        constexpr std::array<std::uint8_t,8> Frame::getData() const noexcept{
            return data_;
        }


    }


}




#endif //RMD_DRIVER_FRAME_HPP
