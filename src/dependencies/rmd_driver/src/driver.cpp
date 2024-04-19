//
// Created by gj on 24. 3. 4.
//

#include "driver.hpp"



namespace rmd_driver{


    Driver::Driver(std::string const& ifname)
            : Node{ifname} {
                printf("Init rmd driver\n");
        return;
    }


    void Driver::addMotor(std::uint8_t actuator_id){
        updateIds(actuator_id);
    }

    void Driver::MotorRunning(std::uint8_t actuator_id){
        MotorRunningRequest const request {};
        auto const response {sendRecv<MotorRunningResponse>(actuator_id,request)};
        return;
    }


    Feedback Driver::sendTorqueSetpoint(std::uint8_t actuator_id,float const current) {
        SetTorqueRequest const request {current};
        auto const response {sendRecv<SetTorqueResponse>(actuator_id,request)};
        return response.getStatus();
    }

    void Driver::shutdownMotor(std::uint8_t actuator_id) {
        ShutdownMotorRequest const request {};
        auto const response {sendRecv<ShutdownMotorResponse>(actuator_id,request)};
        return;
    }

    void Driver::stopMotor(std::uint8_t actuator_id) {
        StopMotorRequest const request {};
        auto const response {sendRecv<StopMotorResponse>(actuator_id,request)};
        return;
    }


}