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


// int main(int argc,char* argv[])
// {

//     RMD_TORQUE_TEST_SHM.SHM_GETID();



//     float rmd_torque_test_buffer = 0.0f;

//     while(true){



//         RMD_TORQUE_TEST_SHM.SHM_READ(&rmd_torque_test_buffer);
//         printf("test : %f", rmd_torque_test_buffer);

//         sleep(1);


        
     
//     }

// }



#include <zmq.hpp>
#include <array>
#include <iostream>
#include <unistd.h> // For sleep()

int main() {
    zmq::context_t context(1);

    // Socket to send messages
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5555");

    std::array<float, 6> torque = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    while (true) {
        zmq::message_t message(torque.data(), torque.size() * sizeof(float));
        publisher.send(message, zmq::send_flags::none);
        
        std::cout << "Sent torque array." << std::endl;

        sleep(1); // Wait a bit before sending the next message
    }
    return 0;
}


