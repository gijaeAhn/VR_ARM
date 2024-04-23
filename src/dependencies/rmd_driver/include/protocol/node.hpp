//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_NODE_HPP
#define RMD_DRIVER_NODE_HPP
#pragma once

#include "address_offset.hpp"
#include "can/node.hpp"
#include "protocol/request.hpp"
#include "protocol/response.hpp"

#include <vector>
#include <iostream>

namespace rmd_driver {

    class Node: protected can::Node{
        public:

        Node(std::string const&ifname);
        Node() = delete;
        Node(Node const&) = delete;
        Node& operator = (Node const&) = default;
        Node(Node&&) = default;
        Node& operator = (Node&&) = default;
        ~Node();

        /**\fn updateIds
           * \brief
           *    Updates the id as well as the send and receive ids in a consistent manner
           *
           * \param[in] actuator_id
           *    The id of the actuator [0, 32]
        */
        void updateIds(std::uint16_t const actuator_id);

        /**\fn send
         * \brief
         *    Writes a given CAN frame based on the request to the internally saved send_id_
         *
         * \param[in] msg
         *    The message that should be sent to the corresponding send_id_
        */
        inline void send(std::uint32_t const actuator_id, Message const& message);

        /**\fn sendRecv
           * \brief
           *    Writes a given CAN frame based on the request to the internally saved send_id_
           *    and waits for a corresponding reply
           *
           * \tparam RESPONSE_TYPE
           *    Type of the response
           * \param[in] request
           *    Request that should be sent to the corresponding send_id_
           * \return
           *    The parsed response message
        */

        template<typename RESPONSE_TYPE>
        [[nodiscard]]
        inline RESPONSE_TYPE sendRecv(std::uint32_t const actuator_id, Message const& message);


        private:

        std::vector<std::uint16_t> actuator_id_list_;
        std::vector<std::uint16_t> can_send_id_list_;
        std::vector<std::uint16_t> can_receive_id_list_;


    };


    Node::Node(std::string const& ifname)
    : can::Node{ifname} {
        return;
    }

    Node::~Node(){
    std::cout << "Deconstruction of node" << std::endl;
    }

    void Node::updateIds(std::uint16_t const actuator_id) {
        if ((actuator_id < 1) || (actuator_id > 32)) {
            throw Exception("Given actuator id '" + std::to_string(actuator_id) + "' out of range [1, 32]!");
        }
        printf("%x\n",id_offset_+actuator_id);
        actuator_id_list_.push_back(actuator_id);
        can_send_id_list_.push_back( id_offset_ + actuator_id) ;
        can_receive_id_list_.push_back( id_offset_ + actuator_id);
        setRecvFilter(can_receive_id_list_[actuator_id -1]);

        printf("%x",can_send_id_list_[actuator_id-1]);
        return;
    }

    void Node::send(std::uint32_t const actuator_id, Message const& msg) {
        write(can_send_id_list_[actuator_id-1], msg.getData());
        return;
    }

    template<typename RESPONSE_TYPE>
    RESPONSE_TYPE Node::sendRecv(std::uint32_t const actuator_id, Message const& message) {

        write(can_send_id_list_[actuator_id -1], message.getData());
        can::Frame const frame {can::Node::read(can_receive_id_list_[actuator_id-1])};
        RESPONSE_TYPE const response {frame.getData()};
        return response;
    }
}


#endif //RMD_DRIVER_NODE_HPP
