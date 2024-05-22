//
// Created by gj on 24. 2. 29.
//
#ifndef RMD_DRIVER_CAN_NODE_HPP
#define RMD_DRIVER_CAN_NODE_HPP
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <mutex>

#include <cstdint>
#include "can/frame.hpp"
#include "can/utilities.hpp"
#include "can/can_exception.hpp"

namespace rmd_driver{
    namespace can{

        class Node{
            public:


            Node(std::string const& ifname);
            Node() = delete;
            Node(Node const&) = delete;
            Node& operator = (Node const&) = default;
            Node(Node&&) = default;
            Node& operator = (Node&&) = default;
            ~Node();

            /**\fn setLoopback
             * \brief
             *    Set the socket to also receive its own messages, this can be desirable for debugging
             *
             * \param[in] is_loopback
             *    If set to true the node will also receive its own messages
            */
            void setLoopback(bool const is_loopback);

            /**\fn setRecvFilter
             * \brief
             *    Set a filter for receiving CAN frames only for specific IDs
             *
             * \param[in] can_id
             *    The CAN id that should be accepted (discarded if \p is_invert set to true)
             * \param[in] is_invert
             *    Invert the CAN id filter: If set to true all messages of the given ID are discarded
            */
            void setRecvFilter(std::uint32_t const& can_id, bool const is_invert = false);

            /**\fn setSendTimeout
             * \brief
             *    Set socket timeout for sending frames
             *
             * \param[in] timeout
             *    Timeout that the socket should be set to for sending frames
            */
            void setSendTimeout(std::chrono::microseconds const& timeout);

            /**\fn setRecvTimeout
             * \brief
             *    Set socket timeout for receiving frames
             *
             * \param[in] timeout
             *    Timeout that the socket should be set to for receiving frames
            */
            void setRecvTimeout(std::chrono::microseconds const& timeout);

            /**\fn setErrorFilters
             * \brief
             *    Set error filters for the socket. We will only receive error frames if we explicitly activate it!
             *
             * \param[in] is_signal_errors
             *    Boolean flags indicating whether errors should be received or not
            */
            void setErrorFilters(bool const is_signal_errors);

            /**\fn read
             * \brief
             *    Read a CAN frame in a blocking manner
             *    Only CAN frames that a receive filter was set for can be read
             *
             * \return
             *    The read CAN frame
            */

            [[nodiscard]]
            Frame read(uint32_t const actuator_id) const;

            void write(Frame const& frame);

            void write(std::uint32_t const can_id, std::array<std::uint8_t,8> const& data);


            protected:

            void initSocket(std::string const& ifname);

            void closeSocket() noexcept;


            std::string ifname_;
            int socket_;
        };
    }
}


#endif //RMD_DRIVER_NODE_HPP
