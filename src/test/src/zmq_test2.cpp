#include <zmq.hpp>
#include <array>
#include <iostream>
#include <cstring> // For memcpy

float calculateAngle(const std::array<float, 6>& torque) {
    // Placeholder calculation, returns the first element for demonstration
    return torque[0] * 2;
}

int main() {
    zmq::context_t context(1);

    // Socket to receive messages on
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5555");
    // subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    zmq::sockopt::array_option<ZMQ_SUBSCRIBE,1> test_sockpot;
    subscriber.set(test_sockpot,"");

    while (true) {
        zmq::message_t request;
        zmq::recv_result_t recv_result = subscriber.recv(request, zmq::recv_flags::none);
        
        if(recv_result == -1){
            std::perror("Message has been corrupted");
        }
        
        std::array<float, 6> receivedTorque;
        memcpy(receivedTorque.data(), request.data(), request.size());
        
        float angle = calculateAngle(receivedTorque);
        std::cout << "Calculated angle based on received torque: " << angle << std::endl;
    }
    return 0;
}
