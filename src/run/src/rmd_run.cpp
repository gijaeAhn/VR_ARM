//
// Created by gj on 24. 3. 5.
//




#include "dependencies/rmd_driver/include/driver.hpp"
#include "dependencies/rmd_driver/include/feedback.hpp"
#include "dependencies/rmd_driver/include/meta/motorParam.hpp"


#include "utilities/include/shm.hpp"
#include "utilities/include/timer.hpp"



#include <stdint.h>
#include <string>
#include <vector>
#include <cstdint>
#include <array>
#include <signal.h>
#include <utility>
#include <algorithm>
#include <csignal>

#define MOTOR_ID_OFFSET 1

utilities::memory::SHM<float>           RMD_TORQUE_SHM(RMD_MOTOR_KEY,RMD_MOTOR_SIZE);
utilities::memory::SHM<std::uint8_t>    RMD_DEBUG_SHM(RMD_DEBUG_KEY,RMD_MOTOR_SIZE);
utilities::memory::SHM<float>           RMD_ANGLE_SHM(RMD_ANGLE_KEY,RMD_MOTOR_SIZE);

void signalHandler(int signum);


int main(int argc, char *argv[])
{   
    std::string driver_name(argv[1]);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM,signalHandler);

    utilities::Timer timer;
    timer.next_execution = std::chrono::steady_clock::now();
    rmd_driver::Driver driver(driver_name);

    std::array<rmd_driver::Feedback,RMD_MOTOR_SIZE> bufFeed;
    std::array<uint8_t,RMD_MOTOR_SIZE> currentShaft{0};
    std::array<uint8_t,RMD_MOTOR_SIZE> previousShaft{0};
    std::array<float, RMD_MOTOR_SIZE> shaftChange{};

    // BUFFER FOR SHM
    float pidBuffer[RMD_MOTOR_SIZE];
    float gravBuffe[RMD_MOTOR_SIZE];
    float sumBuffer[RMD_MOTOR_SIZE];
    float angBuffer[RMD_MOTOR_SIZE];
    float velBuffer[RMD_MOTOR_SIZE];
    

    // INIT MOTORS
    // SLEEP MOTOR * MOTOR INIT TIME
    for (std::size_t index = 0; index < ROBOT_MOTOR_SIZE; ++index) {
        driver.addMotor(index + MOTOR_ID_OFFSET);
        driver.MotorRunning(index + MOTOR_ID_OFFSET);
        auto buf = rmd_driver::Feedback{driver.sendTorqueSetpoint(index + MOTOR_ID_OFFSET, 0)};
        previousShaft[index] = buf.shaft_angle; // Make sure previousShaft has at least ROBOT_MEM_SIZE elements
        angBuffer[index] = 0.0f;
        velBuffer[index] = 0.0f;
        shaftChange[index] = 0.0f;
        sleep(MOTORINIT_TIME);
    }
    


    RMD_TORQUE_SHM.SHM_CREATE();
    RMD_DEBUG_SHM.SHM_CREATE();
    RMD_DEBUG_SHM.SHM_CREATE();

    

    //READ HOME ANGLE CONFIGURATION
    RMD_ANGLE_SHM.SHM_READ(angBuffer);

    //FOR REALTIME FREQ CALC
    std::uint16_t freqCalc = 0;
    std::uint16_t freqSample =  300;
    auto cycleStarttime = std::chrono::steady_clock::now(); 

    //System should be seized when signal come in
    while(true){
        freqCalc ++;
        RMD_TORQUE_SHM.SHM_READ(sumBuffer);

            for(std::uint8_t index; index < RMD_MOTOR_SIZE; index++)
            {
                sumBuffer[index] = std::clamp(sumBuffer[index], -10.0f, 10.0f);
                bufFeed[index] = driver.sendTorqueSetpoint(index+MOTOR_ID_OFFSET,sumBuffer[index]);
                previousShaft[index] = currentShaft[index];
                currentShaft[index] = bufFeed[index].shaft_angle;

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
                
                angBuffer[index] += shaftChange[index]/(rmd_driver::maxShaftAngle)*(rmd_driver::oneShaftCycle);

            }

        RMD_ANGLE_SHM.SHM_WRITE(angBuffer);

        if (freqCalc >= freqSample)
        {
            auto cycleEndtime = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = cycleEndtime - cycleStarttime;

            double frequency = static_cast<double>(freqSample) / elapsed.count(); // Calculate frequency
            std::cout << "Loop Frequency: " << frequency << " Hz" << std::endl;

            // Reset the counter and the start time for the next measurement
            freqCalc = 0;
            cycleStarttime = std::chrono::steady_clock::now();
        }
    }

    return 0;
}


void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << " ) recieved.\n" << std::endl; 
    RMD_TORQUE_SHM.SHM_FREE();
    RMD_DEBUG_SHM.SHM_FREE();
    RMD_ANGLE_SHM.SHM_FREE();
    std::exit(signum);
}






