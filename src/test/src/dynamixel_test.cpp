//
// Created by gj on 24. 3. 5.
//

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0


//SDK
#include "dependencies/dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"
//Utilities
#include "utilities/include/shm.hpp"
#include "utilites/include/timer.hpp"


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


//////// DYNAMIXEL CONFIGURATION

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_LED_RED                563
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Data Byte Length
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#3 ID: 3

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

///////





int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}


int main(int argc, char* argv[]){

    // INPUT PARAM
    // #1 Device Name (USB Location)
    // #2 BaudRate
    // #3 Motor Number

    if(argc < 3) {
        fprintf(stderr, "Usage: %s <Device Name> <BaudRate>\n", argv[0]);
        return 1; // Exit with an error code
    }

    char* DEVICENAME = argv[1];
    std::uint16_t BAUDRATE = static_cast<std::uint16_t>(atoi(argv[2])); 
    std::uint8_t  MOTORNUM = static_cast<std::uint8_t>(atoi(argv[3]));


    //DYNAMIXEL INIT
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);
    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);


    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    bool dxl_getdata_result = false;                // GetParam result
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};   

    uint8_t dxl_error = 0;                          // Dynamixel error
    uint8_t dxl_led_value[2] = {0x00, 0xFF};        // Dynamixel LED value for write
    uint8_t param_goal_position[4];
    int32_t dxl1_present_position = 0;              // Present position
    uint8_t dxl2_led_value_read;                    // Dynamixel LED value for read

    // Open port
    if (portHandler->openPort())
    {
      printf("Succeeded to open the port!\n");
    }
    else
    {
      printf("Failed to open the port!\n");
      printf("Press any key to terminate...\n");
      getch();
      return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate!\n");
    }
    else
    {
      printf("Failed to change the baudrate!\n");
      printf("Press any key to terminate...\n");
      getch();
      return 0;
    }

    // Enable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
      printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }   


    // Enable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
      printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }


    // Add parameter storage for Dynamixel#1 present position
    dxl_addparam_result = groupBulkRead.addParam(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL1_ID);
      return 0;
    }
    // Add parameter storage for Dynamixel#2 LED value
    dxl_addparam_result = groupBulkRead.addParam(DXL2_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL2_ID);
      return 0;
    }




    while(1)
      {
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE)
          break;

        // Allocate goal position value into byte array
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

        // Add parameter storage for Dynamixel#1 goal position
        dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
        if (dxl_addparam_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL1_ID);
          return 0;
        }

        // Add parameter storage for Dynamixel#2 LED value
        dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED, &dxl_led_value[index]);
        if (dxl_addparam_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", DXL2_ID);
          return 0;
        }

        // Bulkwrite goal position and LED value
        dxl_comm_result = groupBulkWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

        // Clear bulkwrite parameter storage
        groupBulkWrite.clearParam();

        do
        {
          // Bulkread present position and LED status
          dxl_comm_result = groupBulkRead.txRxPacket();
          if (dxl_comm_result != COMM_SUCCESS)
          {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
          }
          else if (groupBulkRead.getError(DXL1_ID, &dxl_error))
          {
            printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
          }
          else if (groupBulkRead.getError(DXL2_ID, &dxl_error))
          {
            printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
          }

          // Check if groupbulkread data of Dynamixel#1 is available
          dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
          if (dxl_getdata_result != true)
          {
            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL1_ID);
            return 0;
          }

          // Check if groupbulkread data of Dynamixel#2 is available
          dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
          if (dxl_getdata_result != true)
          {
            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL2_ID);
            return 0;
          }

          // Get present position value
          dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

          // Get LED value
          dxl2_led_value_read = groupBulkRead.getData(DXL2_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED);

          printf("[ID:%03d] Present Position : %d \t [ID:%03d] LED Value: %d\n", DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_led_value_read);

        }while(abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD);

        // Change goal position
        if (index == 0)
        {
          index = 1;
        }
        else
        {
          index = 0;
        }
      }

      // Disable Dynamixel#1 Torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }

      // Disable Dynamixel#2 Torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }

      // Close port
      portHandler->closePort();

      return 0;
}













