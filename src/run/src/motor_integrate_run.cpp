//
// Created by GJ on 5/14/24.
//

#include "driver.hpp"
#include "feedback.hpp"
#include "meta/motorParam.hpp"
#include "dependencies/dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"


#include "utilities/include/timer.hpp"
#include "utilities/include/address.hpp"


#include <stdint.h>
#include <string>
#include <vector>
#include <cstdint>
#include <array>
#include <signal.h>
#include <utility>
#include <algorithm>
#include <csignal>
#include <zmq.hpp>
#include <array>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <zmq.hpp>
#include <cstring>
#include <sstream>
#include <thread>
#include <memory>

#include <fcntl.h>
#include <termios.h>
//SDK
#include "dependencies/dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"
//Utilities
#include "utilities/include/address.hpp"
#include "utilities/include/timer.hpp"
//ZMQ
#include "zmq.hpp"


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <cstdint>
#include <array>
#include <signal.h>
#include <utility>
#include <algorithm>
#include <csignal>
#include <cstdint>
#include <math.h>
#include <mutex>

//////// RMD CONFIGURATION

#define MOTOR_ID_OFFSET 1
#define MOTOR_INIT_TIME 1
#define DEBUQ_FREQ 500



//////// DYNAMIXEL CONFIGURATION

// Control table address

#define ADDR_TORQUE_ENABLE              64
#define ADDR_CURRENT_GOAL               102
#define ADDR_PRESENT_POSITION           132


// Data Byte Length
#define LEN_CURRENT_GOAL                2
#define LEN_PRESENT_POSITION            4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 11
#define DXL2_ID                         2                   // Dynamixel#2 ID: 12
#define DXL3_ID                         13                   // Dynamixel#3 ID: 13

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define TORQUE_CONSTANT                 2.12
#define CURRENT_UNIT                    2.69

#define ESC_ASCII_VALUE                 0x1b
#define DYNA_DEVICE_NAME                "/dev/ttyUSB0"
#define RMD_DEVICE_NAME                 "can0"





std::shared_ptr<rmd_driver::Driver> rmdDriver(nullptr);
std::shared_ptr<dynamixel::PacketHandler> dynaPacketHandler(nullptr);
std::shared_ptr<dynamixel::PortHandler> dynaPortHandler(nullptr);
std::shared_ptr<dynamixel::GroupBulkRead> dynaGroupBulkRead(nullptr, nullptr);
std::shared_ptr<dynamixel::GroupBulkWrite> dynaGroupBulkWrite(nullptr, nullptr);

void rmd_thread_func();
void dyna_thread_func();




int main(int argc, char* argv){
    std::cout << "Start Motor Run" << std::endl;

    rmdDriver = std::make_shared<rmd_driver::Driver>(RMD_DEVICE_NAME);
    dynaPacketHandler = std::make_shared<dynamixel::PacketHandler>(PROTOCOL_VERSION);
    dynaPortHandler = std::make_shared<dynamixel::PortHandler>(DEVICE_NAME);
    dynaGroupBulkWrite = dynamixel::GroupBulkWrite(dynaPortHandler.get(),dynaPacketHandler.get());
    dynaGroupBulkRead = dynamixel::GroupBulkRead(dynaPortHandler.get(),dynaPacketHandler.get());


    //ZMQ Node SET
    zmq::context_t context(1);
    zmq::socket_t motor_torque_subscriber(context,ZMQ_SUB);

    //ZMQ Option SET
    std::stringstream mts;
    std::string host = "localhost";
    mts << "tcp://" << host << ":" <<  MOTOR_ADDR;
    motor_torque_subscriber.connect(mts.str());
    zmq::sockopt::array_option<ZMQ_SUBSCRIBE,1> sockopt;
    motor_torque_subscriber.set(sockopt,"");

    std::array<float,ROBOT_MOTOR_SIZE> robotTorque;
    std::array<float,ROBOT_ANGLE_ADDR> robotAngle;
    robotTorque.fill(0);
    robotAngle.fill(0);
    //Should be init with INIT START

    //DYNA SETTING
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    bool dxl_getdata_result = false;                // GetParam result
    uint8_t dxl_error = 0;                          // Dynamixel error

    std::array<float,DYNAMIXEL_MOTOR_SIZE> Buffer;
    std::array<float,DYNAMIXEL_MOTOR_SIZE> angBuffer;
    std::array<float,DYNAMIXEL_MOTOR_SIZE> currentBuffer;
    std::array<int16_t,DYNAMIXEL_MOTOR_SIZE> unitCurrentBuffer;
    std::array<uint32_t,DYNAMIXEL_MOTOR_SIZE> unitAngBuffer;
    std::array<uint32_t,DYNAMIXEL_MOTOR_SIZE> homePosition;
    sumBuffer.fill(0);
    angBuffer.fill(0);
    currentBuffer.fill(0);
    unitAngBuffer.fill(0);
    homePosition.fill(0);

    //RMD SETTING
    std::uint16_t freqCalc = 0;
    std::uint16_t freqSample = DEBUQ_FREQ;
    std::array<rmd_driver::Feedback,RMD_MOTOR_SIZE> bufFeed;
    std::array<float,RMD_MOTOR_SIZE> currentShaft;
    std::array<float,RMD_MOTOR_SIZE> previousShaft;
    std::array<float,RMD_MOTOR_SIZE> shaftChange;
    std::array<float,RMD_MOTOR_SIZE> rmdTorqueBuffer;
    currentShaft.fill(0);
    previousShaft.fill(0);
    shaftChange.fill(0);
    torqueBuffer.fill(0);
    std::int16_t sendTorque = 0;
    std::uint16_t freqCalc = 0;
    std::uint16_t freqSample = DEBUQ_FREQ;


    for (uint8_t index = 0; index < RMD_MOTOR_SIZE; index++) {
        rmdDriver->addMotor(index + MOTOR_ID_OFFSET);
        rmdDriver->MotorRunning(index + MOTOR_ID_OFFSET);
        printf("Motor %d running", index+MOTOR_ID_OFFSET);
        auto buf = rmd_driver::Feedback{driver.sendTorqueSetpoint(index + MOTOR_ID_OFFSET, 0)};
        previousShaft[index] = buf.getShaft(); // Make sure previousShaft has at least ROBOT_MEM_SIZE elements
        sleep(MOTOR_INIT_TIME);
        printf("Debug MOTOR INIT  %d", index);
    }
    //INIT END

    //Threads
    std::thread rmd_thread(rmd_thread_func());
    std::thread dyna_thread(dyna_thread_func());

    //while(getchar() != specific characte)
    while(true) {
        //ZMQ MOTOR TORQUE
        zmq::message_t robot_torque_sub_message(sizeof(float) * ROBOT_MOTOR_SIZE);
        auto torque_ptr = reinterpret_cast<float *>(robot_torque_sub_message.data());
        zmq::recv_result_t recv_result =  motor_torque_subscriber.recv(robot_torque_sub_message, zmq::recv_flags::none);
        //Checking Recv here??
        if(!(recv_result.has_value() && recv_result.value() > 0 )){
            std::cout << "Failed to recieve torque Array" << std::endl;
        }
            //Debuging ZMQ
        else {
            std::copy(temp_ptr,temp_ptr+ROBOT_MOTOR_SIZE,robotTorque.begin());
            for (const auto& torque : robotTorque) {
                std::cout << torque << " ";
            }
            std::cout << std::endl;

            std::cout << "Torque Size : " << robotTorque.size()  << " message size : "
                      << robot_torque_sub_message.size()
                      << std::endl;
        }

    }
}



void rmd_thread_func(){
    freqCalc += 1;
    for(uint8_t index=0; index < RMD_MOTOR_SIZE; index++){
        rmdTorqueBuffer[index] = std::clamp(rmdTorqueBuffer[index], -10.0f, 10.0f);
        // Motor Constant
        // 9.8T / 3.0 A = 3.267
        // int16t range -2000 ~ 2000
        // to -32A to 32A
        //
        // Torque to unit
        // Torque / 3.267 = Current
        // Sending Value = Torque * 19.131;
        // Rounding Value;
        sendTorque = static_cast<int16_t>(std::round(rmdTorqueBuffer[index] * 19.131f));
        bufFeed[index] = rmdDriver->sendTorqueSetpoint(index+MOTOR_ID_OFFSET,sendTorque);
        previousShaft[index] = currentShaft[index];
        currentShaft[index] = bufFeed[index].getShaft();
        //Estimating current Angle
        auto delta = currentShaft[index] - previousShaft[index];
        if ( delta > 50000) {
            shaftChange[index] = -((rmd_driver::maxShaftAngle - currentShaft[index]) + previousShaft[index]);
        }
        else if (delta < -50000) {
            shaftChange[index] = currentShaft[index] + (rmd_driver::maxShaftAngle - previousShaft[index]);
        }
        else{
            shaftChange[index] = delta;
        }

        robotAngle[index] += shaftChange[index]/(rmd_driver::maxShaftAngle)*(rmd_driver::oneShaftCycle);
    }
    if (freqCalc >= freqSample)
    {
        std::cout << "Recieved Torque : " << sumBuffer[0]  << std::endl;
        auto cycleEndtime = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = cycleEndtime - cycleStarttime;
        float frequency = static_cast<float>(freqSample) / elapsed.count();
        freqCalc = 0;
        std::cout<< "Actual Frequency : " << frequency << std::endl;
        cycleStarttime = std::chrono::steady_clock::now();
    }

}

void dyna_thread_func(){



}
