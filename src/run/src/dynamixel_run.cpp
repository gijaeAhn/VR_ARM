//
// Created by gj on 24. 3. 5.
//

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
#define DEVICE_NAME                     "/dev/ttyUSB0"

/////// Gloabl Variables
dynamixel::PacketHandler* packetHandler = nullptr;
dynamixel::PortHandler* portHandler = nullptr;


int getch(void);
int kbhit(void);
void signalHandler(int signum);











int main(){
    std::signal(SIGINT, signalHandler);

    //ZMQ SETTING
    zmq::context_t context(1);
    zmq::socket_t dyna_torque_subscriber(context,ZMQ_SUB);
    zmq::socket_t dyna_angle_publisher(context,ZMQ_PUB);
    std::stringstream dts;
    std::stringstream dap;
    std::string host = "localhost";
    dts << "tcp://" << host << ":" << DYNAMIXEL_MOTOR_ADDR;
    dap << "tcp://*:" << DYNAMIXEL_ANGLE_ADDR;
    dyna_torque_subscriber.connect(dts.str());
    dyna_angle_publisher.bind(dap.str());
    zmq::sockopt::array_option<ZMQ_SUBSCRIBE,1> sockopt;
    dyna_torque_subscriber.set(sockopt,"");
    zmq::message_t torque_sub_message(sizeof(float)*DYNAMIXEL_MOTOR_SIZE);
    zmq::message_t angle_pub_message(sizeof(float)*DYNAMIXEL_MOTOR_SIZE);
    auto torque_sub_message_ptr = reinterpret_cast<float*>(torque_sub_message.data());
    auto angle_pub_message_ptr = reinterpret_cast<float*>(angle_pub_message.data());
    //ZMQ SETTING END



    //DYNAMIXEL INIT
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);
    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    bool dxl_getdata_result = false;                // GetParam result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint8_t torque_goal[DYNAMIXEL_MOTOR_SIZE][2];
    //DYNAMIXEL INIT END

    //IN USE ARRAYS
    std::array<float,DYNAMIXEL_MOTOR_SIZE> sumBuffer;
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
    //

    // Open port
    if (portHandler->openPort()){
        printf("Succeeded to open the port!\n");
    }
    else{
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(4000000)){
        printf("Succeeded to change the baudrate!\n");
    }
    else{
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }


    for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID + index, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS){
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0){
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }
      else{
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID + index);
      }
    }


    for(uint8_t index = 0 ; index < DYNAMIXEL_MOTOR_SIZE; index++){
      dxl_addparam_result = groupBulkRead.addParam(DXL1_ID + index, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      if (dxl_addparam_result != true){
        fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL1_ID + index);
        return 0;
      }
    }
   
    dxl_comm_result = groupBulkRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (groupBulkRead.getError(DXL1_ID, &dxl_error)){
      printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
    }
    else if (groupBulkRead.getError(DXL2_ID, &dxl_error)){
      printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
    }

    for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE ; index ++){
      dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID + index, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      if (dxl_getdata_result != true){
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL1_ID + index);
        return 0;
      }
    }
   
    for(uint8_t index =0 ; index < DYNAMIXEL_MOTOR_SIZE; index++){
      homePosition[index]= groupBulkRead.getData(DXL1_ID + index, ADDR_PRESENT_POSITION,LEN_PRESENT_POSITION);
    }

    for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
        printf("unit Encoder Motor %d : %d  ",index,homePosition[index]);
        if(index + 1 == DYNAMIXEL_MOTOR_SIZE){
            printf("\n");
        }
    }

   


    int inputch = 0;

    while(inputch != 115){
        //Exit Code Here
        inputch = getch();
        zmq::recv_result_t recv_result = dyna_torque_subscriber.recv(torque_sub_message,zmq::recv_flags::none);

        if(recv_result == -1){
          std::perror("Failed to recieve message");
        }

        std::copy(torque_sub_message_ptr,torque_sub_message_ptr+DYNAMIXEL_MOTOR_SIZE,sumBuffer.data());

        for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
              printf(" Motor Torque %d : %f  ",index+1,sumBuffer[index]);
              if(index + 1 == DYNAMIXEL_MOTOR_SIZE){
                  printf("\n");
              }
        }
        //Torque Constant( Torque / Current ) ~= 2.12 // Suppose it's linear
        //Protocol Unit = 2.69mA
        //Current Limit = 0 ~ 2047
        //Current Range == -Current Limit ~ Current Limit
        //Need to verify the appropriate data type, whether it should be uint16_t or uint32_t
        //
        for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
          unitCurrentBuffer[index] = static_cast<std::int16_t>(((sumBuffer[index] / TORQUE_CONSTANT) * 1000) / CURRENT_UNIT);
          torque_goal[index][0] = DXL_LOBYTE(unitCurrentBuffer[index]);  // Lower byte
          torque_goal[index][1] = DXL_HIBYTE(unitCurrentBuffer[index]);
        }
        for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
          // Add parameter storage for Dynamixel#1 goal position
          dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID + index, ADDR_CURRENT_GOAL, LEN_CURRENT_GOAL, torque_goal[index]);
          if (dxl_addparam_result != true){
            fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL1_ID);
            return 0;
          }
        }
        dxl_comm_result = groupBulkWrite.txPacket();
        if (dxl_comm_result != true) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        groupBulkWrite.clearParam();
        //TX End


        // Bulkread present position and LED status
        dxl_comm_result = groupBulkRead.txRxPacket();
        if (dxl_comm_result != true){
          printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (groupBulkRead.getError(DXL1_ID, &dxl_error)){
          printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
        }
        else if (groupBulkRead.getError(DXL2_ID, &dxl_error)){
          printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
        }

        for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE ; index ++){
          dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID + index, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
          if (dxl_getdata_result != true){
            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL1_ID + index);
            return 0;
          }
        }
       
        for(uint8_t index =0 ; index < DYNAMIXEL_MOTOR_SIZE; index++){
          unitAngBuffer[index] = groupBulkRead.getData(DXL1_ID + index, ADDR_PRESENT_POSITION,LEN_PRESENT_POSITION);
        }

        // Position Unit 0.088 degree
        // EX) 1,044,479 * 0.088 =  91914.152  <255Rev>
        // Compare with Home position  (Current Position Unit - Home Position Unit ) * 0.088  = Current Angle (radian)
        
        for(uint8_t index = 0; index <DYNAMIXEL_MOTOR_SIZE; index++){
          angBuffer[index] =  (static_cast<int32_t>(unitAngBuffer[index]) - static_cast<int32_t>(homePosition[index])) * 0.088f;
        }
          for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
              printf(" Motor Unit Angle %d : %ld  ",index+1,unitAngBuffer[index]);
              if(index + 1 == DYNAMIXEL_MOTOR_SIZE){
                  printf("\n");
              }
          }

        for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
            printf(" Motor Angle %d : %f  ",index+1,angBuffer[index]);
            if(index + 1 == DYNAMIXEL_MOTOR_SIZE){
                printf("\n");
            }
          }

        std::copy(angBuffer.begin(),angBuffer.end(),angle_pub_message_ptr);
        dyna_angle_publisher.send(angle_pub_message,zmq::send_flags::none);
      }
    //END OF LOOP
    //EXIT KEY == "s"

    for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
        unitCurrentBuffer[index] = 0;
        torque_goal[index][0] = DXL_LOBYTE(unitCurrentBuffer[index]);  // Lower byte
        torque_goal[index][1] = DXL_HIBYTE(unitCurrentBuffer[index]);
    }
    for(uint8_t index = 0; index < DYNAMIXEL_MOTOR_SIZE; index++){
        dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID + index, ADDR_CURRENT_GOAL, LEN_CURRENT_GOAL, torque_goal[index]);
        if (dxl_addparam_result != true){
            fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL1_ID + index);
            return 0;
        }
    }
    dxl_comm_result = groupBulkWrite.txPacket();
    portHandler->clearPort();
    portHandler->closePort();
    return 0;
}



int kbhit(void) {
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getch(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    if (kbhit()) {
        ch = getchar();
    } else {
        ch = -1; // No input
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    return ch;
}


