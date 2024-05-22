//
// Created by gj on 24. 3. 5.
//

// Refactor: Update the identifier's name

#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <csignal>

#define  MOTOR_ADDR                 5563
#define  RMD_MOTOR_ADDR             5555
#define  DYNAMIXEL_MOTOR_ADDR       5556

#define  RMD_DEBUG_ADDR             5557 
#define  DYNAMIXEL_DEBUG_ADDR       5558




#define  RMD_ANGLE_ADDR             5559 
#define  DYNAMIXEL_ANGLE_ADDR       5560

#define  TEST_TORQUE_ADDR           5561


#define  ROBOT_ANGLE_ADDR           5561

#define  ROBOT_MOTOR_SIZE              4
#define  MOTORINIT_TIME                0.1f

#define  RMD_MOTOR_SIZE                2

#define  DYNAMIXEL_MOTOR_SIZE          2






