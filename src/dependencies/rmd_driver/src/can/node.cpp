//
// Created by gj on 24. 3. 4.
//

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

#include "can/node.hpp"
#include "can/frame.hpp"
#include "can/utilities.hpp"
#include "can/can_exception.hpp"



struct can_filter rfilter = {0x140, .can_mask = CAN_SFF_MASK };


namespace rmd_driver{
    namespace can{

        Node::Node(std::string const& ifname)
                : ifname_{}, socket_{-1} {
            initSocket(ifname);
            return;
        }

        Node::~Node() {
            closeSocket();
            return;
        }

        void Node::setLoopback(bool const is_loopback) {
            int const recv_own_msgs {static_cast<int>(is_loopback)};
            if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(int)) < 0) {
                throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not configure loopback");
            }
            return;
        }

        void Node::setRecvFilter(std::uint32_t const& can_id, bool const is_invert) {
            struct ::can_filter filter[1] {};
            if (is_invert) {
                filter[0].can_id = can_id | CAN_INV_FILTER;;
            } else {
                filter[0].can_id = can_id;
            }
            filter[0].can_mask = CAN_SFF_MASK;
            if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
                throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not configure read filter");
            }
            return;
        }

        void Node::setSendTimeout(std::chrono::microseconds const& timeout) {
            struct ::timeval const send_timeout {rmd_driver::toTimeval(timeout)};
            if (::setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&send_timeout), sizeof(struct ::timeval)) < 0) {
                throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting socket timeout");
            }
            return;
        }

        void Node::setRecvTimeout(std::chrono::microseconds const& timeout) {
            struct ::timeval const recv_timeout {rmd_driver::toTimeval(timeout)};
            if (::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&recv_timeout), sizeof(struct ::timeval)) < 0) {
                throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting socket timeout");
            }
            return;
        }

        void Node::setErrorFilters(bool const is_signal_errors) {
            ::can_err_mask_t err_mask {};
            if (is_signal_errors) {
                err_mask = (CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB | CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_TRX |
                            CAN_ERR_ACK | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED);
            }
            if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(::can_err_mask_t)) < 0) {
                throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error setting error acknowledgement");
            }
            return;
        }

        Frame Node::read(uint32_t actuator_id) const {
            struct ::can_frame frame {};
            struct ::can_frame error_frame {};
            error_frame.can_id = actuator_id;
            std::array<std::uint8_t,8> error_data {0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

            rfilter.can_id = actuator_id;
            ::setsockopt(socket_,SOL_CAN_RAW,CAN_RAW_FILTER,&rfilter,sizeof(rfilter));
            if (::read(socket_,&frame,sizeof(frame)) < 0) {
                //Type of the exception should be modified.
              throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Could not read CAN frame");
                Frame const fail(actuator_id,error_data,false);
                return fail;}


            std::array<std::uint8_t,8> data {};
            std::copy(std::begin(frame.data), std::end(frame.data), std::begin(data));
            Frame const f{frame.can_id, data};
            return f;
        }

        void Node::write(Frame const& frame) {
            return write(frame.getId(), frame.getData());
        }

        void Node::write(std::uint32_t const can_id, std::array<std::uint8_t,8> const& data) {
            struct ::can_frame frame {};
            frame.can_id = can_id;
            frame.can_dlc = 8;
            std::copy(std::begin(data), std::end(data), std::begin(frame.data));

            float return_frame_size = ::write(socket_,&frame,sizeof(struct ::can_frame));
            // Should check the size of the "return_frame_size"
            // If error occurs, size will be -1

            if(return_frame_size == -1){
                //Type of the exception should be modified.
                throw SocketException(errno, std::generic_category(), "Fail to write on " + ifname_ );
            }

            return;
        }

        void Node::initSocket(std::string const& ifname) {
            ifname_ = ifname;
            socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (socket_ < 0) {
                throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error creating socket");
            }

            struct ::ifreq ifr {};
            std::strcpy(ifr.ifr_name, ifname.c_str());
            if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
                throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error manipulating device parameters");
            }

            struct ::sockaddr_can addr {};
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            if (::bind(socket_, reinterpret_cast<struct ::sockaddr*>(&addr), sizeof(addr)) < 0) {
                throw SocketException(errno, std::generic_category(), "Interface '" + ifname_ + "' - Error assigning address to socket");
            }
            return;
        }

        void Node::closeSocket() noexcept {
        ::close(socket_);
        return;
    }

    }
}
