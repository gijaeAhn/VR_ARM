//
// Created by gj on 5/7/24.
// This file subscribes to headset rotation and controller transform matrix data,
// and publishes headset images. Integration with the Azure Kinect package is essential.


#include "ControllerListener.hpp"

using namespace std::chrono_literals;

ControllerListener::ControllerListener()
        : Node("QuestControllerListener")
{
    // Initialize parameters
    left_controller_frame = this->declare_parameter<std::string>("left_controller_frame", "left_hand");
    right_controller_frame = this->declare_parameter<std::string>("right_controller_frame", "right_hand");

    left_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    left_controller_listener_ = std::make_shared<tf2_ros::TransformListener>(*left_buffer_);

    right_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    right_controller_listener_ = std::make_shared<tf2_ros::TransformListener>(*right_buffer_);

    zmq::context_t context(1);
    zmq_left = std::make_unique<zmq::socket_t>(context, ZMQ_PUB);
    zmq_right = std::make_unique<zmq::socket_t>(context,ZMQ_PUB);

    std::stringstream rhAddress;
    std::stringstream lhAddress;

    rhAddress << "tcp://*" << ":" << LEFT_HAND_ADDR;
    lhAddress << "tcp://*" << ":" << RIGHT_HAND_ADDRR;
    zmq_right->bind(rhAddress.str());
    zmq_left->bind(lhAddress.str());

    // Refactor : period
    timer_ = this->create_wall_timer(5ms, std::bind(&ControllerListener::on_timer, this));

}

void ControllerListener::on_timer()
{
    geometry_msgs::msg::TransformStamped left_temp;
    geometry_msgs::msg::TransformStamped right_temp;
    zmq::message_t left_message;
    zmq::message_t right_message;
    zmq::send_result_t  left_send_result;
    zmq::send_result_t right_send_result;
    // Timer callback implementation
    try {
        left_temp = left_buffer_->lookupTransform(headsetFrame,
                                                 leftHandFrame,
                                                        tf2::TimePointZero);
        left_message = serializeTransform(left_temp);
        left_send_result = zmq_left->send(left_message,zmq::send_flags::none);
    }catch(const tf2::TransformException & ex){
        RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                headsetFrame.c_str(), leftHandFrame.c_str(), ex.what());
        return;
    }
    try {
        right_temp = right_buffer_->lookupTransform(headsetFrame,
                                                  rightHandFrame,
                                              tf2::TimePointZero);
        right_message = serializeTransform(right_temp);
        right_send_result = zmq_right->send(right_message,zmq::send_flags::none);
    }catch(const tf2::TransformException & ex){
        RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                headsetFrame.c_str(), rightHandFrame.c_str(), ex.what());
        return;
    }

    if(left_send_result && right_send_result){
        RCLCPP_INFO(this->get_logger(), "Succeed to send Message");
    }
}

zmq::message_t serializeTransform(const geometry_msgs::msg::TransformStamped& transformStamped) {
    // Extract rotation and translation from the transform
    const auto& rotation = transformStamped.transform.rotation;
    const auto& translation = transformStamped.transform.translation;

    // Prepare a buffer with enough space for rotation (4 doubles: x, y, z, w) and translation (3 doubles: x, y, z)
    size_t size = sizeof(double) * (4 + 3);
    zmq::message_t message(size);

    // Copy data to the buffer
    double* buffer = reinterpret_cast<double*>(message.data());
    buffer[0] = rotation.x;
    buffer[1] = rotation.y;
    buffer[2] = rotation.z;
    buffer[3] = rotation.w;
    buffer[4] = translation.x;
    buffer[5] = translation.y;
    buffer[6] = translation.z;

    return message;
}