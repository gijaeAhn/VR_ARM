#include <zmq.hpp>
#include <array>
#include <iostream>
#include <unistd.h> // For sleep()
#include <chrono>
#include <thread>

int main() {
    zmq::context_t context(1);

    // Socket to send messages
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5555");

    std::array<float, 6> torque = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        
        zmq::message_t message(torque.data(), torque.size() * sizeof(float));
        publisher.send(message, zmq::send_flags::none);
        
        std::cout << "Sent torque array." << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double frequency = 1.0 / elapsed.count();

        std::cout << "Actual frequency: " << frequency << " Hz" << std::endl;
    }
    return 0;
}
