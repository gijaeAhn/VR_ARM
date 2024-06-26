#ifndef CONTROLLER_LISTENER_HPP
#define CONTROLLER_LISTENER_HPP

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
#include "utilities/include/address.hpp"

#ifdef __APPLE__
#include "/opt/homebrew/Caskroom/miniforge/base/envs/ros_env/include/zmq.hpp"
#endif
#ifdef __linux__
#include <zmq.hpp>
#endif



class ControllerListener : public rclcpp::Node
{
public:
    ControllerListener();

private:
    void on_timer();

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> left_controller_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> right_controller_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> left_buffer_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> right_buffer_{nullptr};
    std::string left_controller_frame;
    std::string right_controller_frame;
    std::unique_ptr<zmq::socket_t> zmq_left{nullptr};
    std::unique_ptr<zmq::socket_t> zmq_right{nullptr};

    zmq::message_t serializeTransform(const geometry_msgs::msg::TransformStamped& transformStamped);

    std::string rightHandFrame = "rightHand";
    std::string leftHandFrame = "leftHand";
    std::string headsetFrame = "headset";
};

#endif // CONTROLLER_LISTENER_HPP
