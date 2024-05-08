//
// Created by gj on 5/7/24.
// This file subscribes to headset rotation and controller transform matrix data,
// and publishes headset images. Integration with the Azure Kinect package is essential.


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <zmq.hpp>


using namespace std::chrono_literals;


class ControllerListener : rclcpp::Node
{
public:
    ControllerListener()
    : Node("QuestControllerListener")
    //INIT PARAMS
    {
        left_controller_frame = this->declare_parameter<std::string>("left_controller_frame", "left_hand");
        right_controller_frame = this->declare_parameter<std::string>("right_controller_frame", "right_hand");
        left_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
        left_controller_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*left_buffer_);
        right_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
        right_controller_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*right_buffer_);
        zmq::context_t context(1);
        zmq_ = std::make_unique<zmq::socket_t>(context,ZMQ_PUB);
        timer_ = this->create_wall_timer(
                1s, std::bind(&ControllerListener::on_timer, this));

    }
private:
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> left_controller_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> right_controller_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> left_buffer_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> right_buffer_{nullptr};
    std::string left_controller_frame;
    std::string right_controller_frame;

    std::unique_ptr<zmq::socket_t> zmq_{nullptr};

    void on_timer(){

    }


};

//
