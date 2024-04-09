// //
// // Created by gj on 24. 4. 3.
// //





// #include "utilities/include/shm.hpp"

// #include <vector>
// #include <iostream>
// #include <csignal>
// #include <algorithm>
// #include <cstdint>

// utilities::memory::SHM<float> RMD_TORQUE_TEST_SHM(RMD_MOTOR_KEY,RMD_MOTOR_SIZE);



// void signalHandler(int signal);


// int main(int argc,char* argv[])
// {


//     RMD_TORQUE_TEST_SHM.SHM_GETID();


//     float rmd_torque_test_buffer[RMD_MOTOR_KEY] = {0};

//     while(true){



//         rmd_torque_test_buffer[0] += 1;

//         RMD_TORQUE_TEST_SHM.SHM_WRITE(rmd_torque_test_buffer);
//         printf("Torque Sending \n");

        
     
//     }

// }


#include <zmq.hpp>
#include <array>
#include <iostream>
#include <cstring> // For memcpy

int calculateAngle(const std::array<float, 6>& torque) {
    // Placeholder calculation, returns the first element for demonstration
    return torque[0] * 2;
}

int main() {
    zmq::context_t context(1);

    // Socket to receive messages on
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5555");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    while (true) {
        zmq::message_t request;
        subscriber.recv(request, zmq::recv_flags::none);
        
        std::array<float, 6> receivedTorque;
        memcpy(receivedTorque.data(), request.data(), request.size());
        
        float angle = calculateAngle(receivedTorque);
        std::cout << "Calculated angle based on received torque: " << angle << std::endl;
    }
    return 0;
}
