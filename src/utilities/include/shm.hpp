//
// Created by gj on 24. 3. 5.
//


#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <csignal>

#define  MOTOR_ADDR                 5554
#define  RMD_MOTOR_ADDR             5555
#define  DYNAMIXEL_MOTOR_ADDR       5556

#define  RMD_DEBUG_ADDR             5557 
#define  DYNAMIXEL_DEBUG_ADDR       5558


#define  RMD_MOTOR_SIZE                1    
#define  DYNAMIXEL_MOTOR_SIZE          1    

#define  RMD_ANGLE_ADDR             5559 
#define  DYNAMIXEL_ANGLE_ADDR       5560


#define  ROBOT_ANGLE_ADDR           5561

#define  ROBOT_MOTOR_SIZE              6
#define  MOTORINIT_TIME                0.1f






